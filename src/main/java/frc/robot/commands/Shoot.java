package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Elastic;

/**
 * Shoot — aims at the goal, spins up the shooter, and feeds game pieces.
 *
 * In teleop mode (isAuto = false): runs until interrupted (button released),
 * or self-terminates when the hub is imminently going inactive.
 * Aborts early if the robot is outside its alliance shooting boundary or
 * the alliance hub is currently inactive (teleop only).
 *
 * In auto mode (isAuto = true): self-terminates when the hopper is empty
 * or a hard timeout fires.
 *
 * ── Empty detection ───────────────────────────────────────────────────────────
 * Empty is detected by watching the Kraken's stator current. When a game piece
 * loads the flywheel, stator current spikes above kShotCurrentThreshold.
 * To avoid acting on a single noisy reading, kShotSampleCount consecutive
 * above-threshold samples are required before a shot is confirmed. Once at
 * least one shot has been detected (m_shotEverDetected), if current stays
 * below the threshold continuously for the settle window we call the hopper
 * empty. Detection is gated on atTargetRPM() to ignore the spinup current spike.
 *
 * If no game pieces were ever fed (bad auto path, failed intake, etc.),
 * m_shotEverDetected stays false and the command falls through to the hard timeout.
 *
 * Two settle windows apply:
 *   Inside expected window  (elapsed >= expectedTime - kEarlyWindowSecs): kEmptySettleTime_InWindow
 *   Outside expected window (before that gate opens):                     kEmptySettleTime_OutWindow
 * Hard timeout fires at: (expectedShootTimeSecs + kLateWindowSecs)
 *
 * ── Shift schedule ────────────────────────────────────────────────────────────
 * Game data (sent ~3 s after auto ends) encodes which alliance's hub is FIRST
 * inactive, determining which of two shift schedules applies for the match.
 *
 * All times are match-timer countdown values (seconds remaining).
 *
 *   RED_FIRST_INACTIVE schedule (gameData.charAt(0) == 'R'):
 *     Shift 1 (2:10–1:45)  Red INACTIVE, Blue active
 *     Shift 2 (1:45–1:20)  Red active,   Blue INACTIVE
 *     Shift 3 (1:20–0:55)  Red INACTIVE, Blue active
 *     Shift 4 (0:55–0:30)  Red active,   Blue INACTIVE
 *
 *   BLUE_FIRST_INACTIVE schedule (gameData.charAt(0) == 'B'):
 *     Shift 1 (2:10–1:45)  Red active,   Blue INACTIVE
 *     Shift 2 (1:45–1:20)  Red INACTIVE, Blue active
 *     Shift 3 (1:20–0:55)  Red active,   Blue INACTIVE
 *     Shift 4 (0:55–0:30)  Red INACTIVE, Blue active
 *
 * Outside these windows (AUTO, TRANSITION, END GAME) both hubs are active.
 */
public class Shoot extends Command {

    // ── Shift schedule ────────────────────────────────────────────────────────

    /**
     * A single shift window: time range (countdown seconds) and which alliance
     * hub is INACTIVE during that window.
     */
    private static final class ShiftWindow {
        final double high;      // countdown seconds at window START (higher value)
        final double low;       // countdown seconds at window END   (lower value)
        final Alliance inactive;

        ShiftWindow(double high, double low, Alliance inactive) {
            this.high     = high;
            this.low      = low;
            this.inactive = inactive;
        }
    }

    /**
     * Shift schedule when Red is the FIRST inactive alliance.
     * gameData.charAt(0) == 'R'
     */
    private static final ShiftWindow[] SCHEDULE_RED_FIRST = {
        new ShiftWindow(130.0, 105.0, Alliance.Red),   // Shift 1
        new ShiftWindow(105.0,  80.0, Alliance.Blue),  // Shift 2
        new ShiftWindow( 80.0,  55.0, Alliance.Red),   // Shift 3
        new ShiftWindow( 55.0,  30.0, Alliance.Blue),  // Shift 4
    };

    /**
     * Shift schedule when Blue is the FIRST inactive alliance.
     * gameData.charAt(0) == 'B'
     */
    private static final ShiftWindow[] SCHEDULE_BLUE_FIRST = {
        new ShiftWindow(130.0, 105.0, Alliance.Blue),  // Shift 1
        new ShiftWindow(105.0,  80.0, Alliance.Red),   // Shift 2
        new ShiftWindow( 80.0,  55.0, Alliance.Blue),  // Shift 3
        new ShiftWindow( 55.0,  30.0, Alliance.Red),   // Shift 4
    };

    /**
     * Returns true if the given alliance's hub is currently INACTIVE.
     *
     * Before game data arrives (empty string) or outside all shift windows
     * (AUTO, TRANSITION, END GAME) both hubs are active → returns false.
     *
     * @param alliance the alliance to check
     */
    private static boolean isHubInactive(Alliance alliance) {
        String gameData = DriverStation.getGameSpecificMessage();

        // Game data not yet available — both hubs active
        if (gameData == null || gameData.isEmpty()) return false;

        ShiftWindow[] schedule;
        char firstInactive = gameData.charAt(0);
        if (firstInactive == 'R') {
            schedule = SCHEDULE_RED_FIRST;
        } else if (firstInactive == 'B') {
            schedule = SCHEDULE_BLUE_FIRST;
        } else {
            // Unexpected FMS value — assume active
            return false;
        }

        double matchTime = DriverStation.getMatchTime();

        for (ShiftWindow window : schedule) {
            if (matchTime <= window.high && matchTime > window.low) {
                return window.inactive == alliance;
            }
        }

        // Outside all shift windows (AUTO, TRANSITION, END GAME) → both active
        return false;
    }

    /**
     * Returns true if the given alliance's hub will become inactive within
     * {@code bufferSecs} seconds, or is already inactive now.
     * Used in teleop to stop feeding before in-flight fuel would reach a dead hub.
     */
    private static boolean isHubImminentlyInactive(Alliance alliance, double bufferSecs) {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) return false;

        ShiftWindow[] schedule;
        char firstInactive = gameData.charAt(0);
        if (firstInactive == 'R') {
            schedule = SCHEDULE_RED_FIRST;
        } else if (firstInactive == 'B') {
            schedule = SCHEDULE_BLUE_FIRST;
        } else {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();

        for (ShiftWindow window : schedule) {
            if (matchTime <= window.high && matchTime > window.low) {
                // Inside a window — hub is already inactive right now
                return window.inactive == alliance;
            }
            // Check if we're about to enter a window where this alliance goes inactive
            if (window.inactive == alliance
                    && matchTime > window.high
                    && matchTime <= window.high + bufferSecs) {
                return true;
            }
        }
        return false;
    }

    // ── Dependencies ──────────────────────────────────────────────────────────

    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final boolean m_isAuto;

    /**
     * Expected duration (seconds) to drain the full hopper.
     * Only used in auto mode; ignored in teleop.
     */
    private final double m_expectedShootTimeSecs;

    // ── Runtime state ─────────────────────────────────────────────────────────

    private final Timer m_commandTimer  = new Timer();
    private final Timer m_emptyTimer    = new Timer();
    private boolean m_emptyTimerRunning = false;
    private double  m_emptySettleTime   = kShooter.kEmptySettleTime_OutWindow;

    /**
     * Latches true the first time a confirmed shot is detected (kShotSampleCount
     * consecutive above-threshold current samples).
     * Prevents early exit if no game pieces were ever fed — in that case the
     * command falls through to the hard timeout instead.
     */
    private boolean m_shotEverDetected = false;

    /**
     * Rolling count of consecutive execute() ticks where stator current has
     * exceeded kShotCurrentThreshold. Resets to 0 whenever current drops below
     * the threshold. A shot is confirmed once this reaches kShotSampleCount.
     */
    private int m_shotSampleCount = 0;

    private final SwerveRequest.FieldCentricFacingAngle m_fieldCentricFacingAngle =
        new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(5.0, 0, 0);
    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    private Alliance m_alliance;
    private Translation2d m_goalPos;

    /** Set in initialize(); false triggers early exit in teleop. */
    private boolean m_preflightPassed = false;

    // ── Constructors ──────────────────────────────────────────────────────────

    /** Teleop constructor — runs until interrupted or hub goes imminently inactive. */
    public Shoot(Shooter shooter, Feeder feeder, CommandSwerveDrivetrain drivetrain) {
        this(shooter, feeder, drivetrain, false, 0.0);
    }

    /**
     * Full constructor.
     *
     * @param shooter               shooter subsystem
     * @param feeder                feeder subsystem
     * @param drivetrain            swerve drivetrain
     * @param isAuto                if true, self-terminates when hopper is empty
     * @param expectedShootTimeSecs estimated seconds to drain the full hopper;
     *                              only used when isAuto is true
     */
    public Shoot(
        Shooter shooter,
        Feeder feeder,
        CommandSwerveDrivetrain drivetrain,
        boolean isAuto,
        double expectedShootTimeSecs
    ) {
        m_shooter = shooter;
        m_feeder  = feeder;
        m_drivetrain = drivetrain;
        m_isAuto = isAuto;
        m_expectedShootTimeSecs = expectedShootTimeSecs;

        addRequirements(shooter, feeder, drivetrain);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_commandTimer.restart();
        m_emptyTimer.reset();
        m_emptyTimerRunning = false;
        m_emptySettleTime   = kShooter.kEmptySettleTime_OutWindow;
        m_shotEverDetected  = false;
        m_shotSampleCount   = 0;

        // Cache alliance and goal once — these don't change mid-match.
        // If the FMS alliance is unavailable, fall back to the closest hub
        // rather than silently defaulting to a hard-coded alliance.
        m_alliance = DriverStation.getAlliance().orElseGet(() -> closestAlliance(
            m_drivetrain.getState().Pose.getTranslation()
        ));
        m_goalPos  = (m_alliance == Alliance.Red) ? kShooter.kRedGoal : kShooter.kBlueGoal;

        // Set shooter distance once from the starting position.
        // In auto the robot should be stationary at shoot time;
        // in teleop the driver is expected to be in position before pressing.
        Translation2d initPos = m_drivetrain.getState().Pose.getTranslation();
        m_shooter.setShootingDistance(initPos.getDistance(m_goalPos));

        // Teleop only: validate boundary and shift before committing
        m_preflightPassed = m_isAuto || checkPreflight(initPos);
    }

    /**
     * Returns whichever alliance's hub the robot is currently closer to.
     * Used as a fallback when the FMS has not yet reported alliance color,
     * e.g. during practice matches or early in the connection sequence.
     */
    private static Alliance closestAlliance(Translation2d robotPos) {
        double distRed  = robotPos.getDistance(kShooter.kRedGoal);
        double distBlue = robotPos.getDistance(kShooter.kBlueGoal);
        return (distRed <= distBlue) ? Alliance.Red : Alliance.Blue;
    }

    /**
     * Validates boundary and shift conditions at the start of a teleop shoot.
     * Posts an Elastic error notification and returns false if any check fails.
     */
    private boolean checkPreflight(Translation2d robotPos) {
        // Boundary check
        double x = robotPos.getX();
        boolean inBoundary = (m_alliance == Alliance.Red)
            ? x >= kShooter.redXBoundary
            : x <= kShooter.blueXBoundary;

        if (!inBoundary) {
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(Elastic.NotificationLevel.ERROR)
                .withTitle("Shoot Blocked")
                .withDescription("Robot is outside the shooting boundary.")
            );
            return false;
        }

        // Shift check
        if (isHubInactive(m_alliance)) {
            Elastic.sendNotification(new Elastic.Notification()
                .withLevel(Elastic.NotificationLevel.ERROR)
                .withTitle("Shoot Blocked")
                .withDescription("Cannot shoot during inactive shift.")
            );
            return false;
        }

        return true;
    }

    @Override
    public void execute() {
        // Teleop: abort immediately if preflight failed
        if (!m_preflightPassed) return;

        // ── 1. Aim at goal using CURRENT pose ─────────────────────────────────
        // Re-computed each tick so the heading target stays accurate as the
        // robot moves (most relevant in auto).
        Translation2d currentPos = m_drivetrain.getState().Pose.getTranslation();

        double targetAngle = Math.atan2(
            m_goalPos.getY() - currentPos.getY(),
            m_goalPos.getX() - currentPos.getX()
        );

        // Use Rotation2d subtraction for a wrap-safe angle error (handles the
        // ±π boundary correctly, e.g. 179° vs -179° gives ~0 not ~360°).
        boolean atAngle = Math.abs(
            m_drivetrain.getState().Pose.getRotation()
                .minus(Rotation2d.fromRadians(targetAngle))
                .getRadians()
        ) < kShooter.angleTolerance_Rads;

        if (atAngle) {
            m_drivetrain.setControl(m_brake);
        } else {
            m_drivetrain.setControl(
                m_fieldCentricFacingAngle
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withTargetDirection(Rotation2d.fromRadians(targetAngle))
            );
        }

        // ── 2. Feed once aimed and up to speed ────────────────────────────────
        if (m_shooter.atTargetRPM() && atAngle) {
            m_feeder.feed();
        } else {
            m_feeder.stop();
        }

        // ── 3. Empty-hopper detection (auto only) ──────────────────────────────
        // Watches Kraken stator current for shot detection. A game piece loading
        // the flywheel causes a clear current spike above kShotCurrentThreshold.
        //
        // Confirmation requires kShotSampleCount consecutive above-threshold
        // readings (currently 3) to avoid acting on a single transient. The
        // sample counter resets to 0 whenever current drops below the threshold,
        // so any gap in readings restarts the confirmation window.
        //
        // Detection is gated on atTargetRPM() to ignore the spinup current spike.
        //
        // Once at least one shot is confirmed, if current stays below the
        // threshold for the settle window we call the hopper empty.
        //
        // If no shots are ever confirmed (bad auto, empty hopper from the start),
        // m_shotEverDetected stays false and we fall through to the hard timeout.
        if (m_isAuto) {
            double elapsed = m_commandTimer.get();
            boolean inDetectionWindow = elapsed >= (m_expectedShootTimeSecs - kShooter.kEarlyWindowSecs);
            m_emptySettleTime = inDetectionWindow
                ? kShooter.kEmptySettleTime_InWindow
                : kShooter.kEmptySettleTime_OutWindow;

            // Only watch current once the flywheel is at speed — avoids treating
            // the spinup current spike as a shot detection event.
            if (m_shooter.atTargetRPM()) {
                if (m_shooter.getStatorCurrent() > kShooter.kShotCurrentThreshold) {
                    m_shotSampleCount++;
                } else {
                    m_shotSampleCount = 0;
                }
            }

            boolean shotConfirmed = m_shotSampleCount >= kShooter.kShotSampleCount;

            if (shotConfirmed) {
                // Confirmed game piece loading — reset empty timer
                m_shotEverDetected  = true;
                m_shotSampleCount   = 0;   // reset so next piece needs its own confirmation
                m_emptyTimer.reset();
                m_emptyTimerRunning = false;
            } else if (m_shotEverDetected) {
                // No shot currently confirmed, but at least one has happened — run empty timer
                if (!m_emptyTimerRunning) {
                    m_emptyTimer.restart();
                    m_emptyTimerRunning = true;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (!m_preflightPassed) return true;

        // Teleop: self-terminate when the hub is imminently going inactive;
        if (!m_isAuto) {
            return isHubImminentlyInactive(m_alliance, kShooter.kTimeToScore);
        }

        boolean hopperEmpty = m_shotEverDetected
            && m_emptyTimerRunning
            && m_emptyTimer.hasElapsed(m_emptySettleTime);
        boolean timedOut = m_commandTimer.get()
            >= (m_expectedShootTimeSecs + kShooter.kLateWindowSecs);

        return hopperEmpty || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
    }
}