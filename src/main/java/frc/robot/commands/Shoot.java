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
import java.util.function.DoubleSupplier;

/**
 * Shoot — aims at the goal, spins up the shooter, and feeds game pieces.
 *
 * Supports SHOOT ON THE MOVE: joystick X/Y inputs are accepted and applied
 * while aiming. Translation speed is capped at kShootOnMoveSpeedLimit (0.5 of
 * MaxSpeed) so the robot stays controllable while the heading PID works.
 *
 * The facing-angle request includes an orbital feedforward term so the heading
 * controller pre-leads the angle target when the robot is moving tangentially
 * around the hub. Without it, the gyro-based heading PID would always lag
 * behind the required angle as the robot sweeps around the goal.
 *
 * Orbital feedforward derivation:
 *   The angle from robot to hub changes at rate:  dθ/dt = v_tangential / r
 *   where v_tangential is the component of robot velocity perpendicular to the
 *   robot→hub vector, and r is the distance to the hub.
 *   We inject this as a rotational feedforward (rad/s) into the swerve request.
 *
 * In teleop mode (isAuto = false): runs until interrupted (button released),
 * or self-terminates when the hub is imminently going inactive.
 * Feeding is gated each tick: if the robot is outside its alliance shooting
 * boundary OR the alliance hub is currently inactive, the feeder is held and
 * the drivetrain continues aiming so the robot is ready the moment conditions
 * clear. The driver may therefore hold the shoot button before crossing into
 * the alliance zone.
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
     * Resolves the shift schedule from the current game-specific message.
     * Returns null if game data is unavailable or contains an unexpected value,
     * which callers treat as "both hubs active".
     */
    private static ShiftWindow[] resolveSchedule() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) return null;
        char firstInactive = gameData.charAt(0);
        if (firstInactive == 'R') return SCHEDULE_RED_FIRST;
        if (firstInactive == 'B') return SCHEDULE_BLUE_FIRST;
        return null; // Unexpected FMS value — assume active
    }

    /**
     * Returns true if the given alliance's hub is currently INACTIVE.
     *
     * Before game data arrives (empty string) or outside all shift windows
     * (AUTO, TRANSITION, END GAME) both hubs are active → returns false.
     *
     * @param alliance the alliance to check
     */
    private static boolean isHubInactive(Alliance alliance) {
        ShiftWindow[] schedule = resolveSchedule();
        if (schedule == null) return false;

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
        ShiftWindow[] schedule = resolveSchedule();
        if (schedule == null) return false;

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
     * Joystick velocity suppliers (meters per second, field-relative).
     * Always {@code () -> 0.0} in auto so the robot holds its shoot position.
     */
    private final DoubleSupplier m_vxSupplier;
    private final DoubleSupplier m_vySupplier;

    /**
     * Maximum robot speed while shoot-on-the-move is active (m/s).
     * Set to 50 % of the robot's full MaxSpeed in RobotContainer.
     */
    private final double m_shootOnMoveSpeedLimit;

    /**
     * Expected duration (seconds) to drain the full hopper.
     * Only used in auto mode; ignored in teleop.
     */
    private final double m_expectedShootTimeSecs;

    // ── Runtime state ─────────────────────────────────────────────────────────

    private final Timer m_commandTimer  = new Timer();
    private final Timer m_emptyTimer    = new Timer();
    private boolean m_emptyTimerRunning;
    private double  m_emptySettleTime;

    /**
     * Latches true the first time a confirmed shot is detected (kShotSampleCount
     * consecutive above-threshold current samples).
     */
    private boolean m_shotEverDetected = false;

    /**
     * Rolling count of consecutive execute() ticks where stator current has
     * exceeded kShotCurrentThreshold. Resets to 0 whenever current drops below
     * the threshold.
     */
    private int m_shotSampleCount = 0;

    /**
     * Plain field-centric request. We compute heading output ourselves (PID + orbital FF)
     * so we can inject the feedforward term as withRotationalRate().
     * FieldCentricFacingAngle manages rotation internally and does not expose that method.
     */
    private final SwerveRequest.FieldCentric m_fieldCentric =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    /**
     * Heading PID: kP=5, kI=0, kD=0. Output units are rad/s, fed into withRotationalRate().
     * Continuous input is enabled over [-π, π] for correct wrap-around behaviour.
     */
    private final edu.wpi.first.math.controller.PIDController m_headingPID =
        new edu.wpi.first.math.controller.PIDController(5.0, 0.0, 0.0);

    {
        // Set once — continuous input over [-π, π] for correct wrap-around behaviour.
        m_headingPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    private Alliance m_alliance;
    private Translation2d m_goalPos;

    // ── Constructors ──────────────────────────────────────────────────────────

    /**
     * Teleop constructor — shoot on the move.
     *
     * @param shooter               shooter subsystem
     * @param feeder                feeder subsystem
     * @param drivetrain            swerve drivetrain
     * @param vxSupplier            field-relative X velocity from joystick (m/s)
     * @param vySupplier            field-relative Y velocity from joystick (m/s)
     * @param shootOnMoveSpeedLimit max translation speed while shooting (m/s);
     *                              pass (MaxSpeed * 0.5) from RobotContainer
     */
    public Shoot(
        Shooter shooter,
        Feeder feeder,
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier,
        double shootOnMoveSpeedLimit
    ) {
        this(shooter, feeder, drivetrain, vxSupplier, vySupplier, shootOnMoveSpeedLimit, false, 0.0);
    }

    /**
     * Full constructor — supports both teleop shoot-on-the-move and auto.
     *
     * @param shooter               shooter subsystem
     * @param feeder                feeder subsystem
     * @param drivetrain            swerve drivetrain
     * @param vxSupplier            field-relative X velocity supplier (m/s);
     *                              may be {@code () -> 0.0} for stationary auto
     * @param vySupplier            field-relative Y velocity supplier (m/s);
     *                              may be {@code () -> 0.0} for stationary auto
     * @param shootOnMoveSpeedLimit max translation speed while shooting (m/s)
     * @param isAuto                if true, self-terminates when hopper is empty
     * @param expectedShootTimeSecs estimated seconds to drain the full hopper;
     *                              only used when isAuto is true
     */
    public Shoot(
        Shooter shooter,
        Feeder feeder,
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier,
        double shootOnMoveSpeedLimit,
        boolean isAuto,
        double expectedShootTimeSecs
    ) {
        m_shooter = shooter;
        m_feeder  = feeder;
        m_drivetrain = drivetrain;
        m_vxSupplier = vxSupplier;
        m_vySupplier = vySupplier;
        m_shootOnMoveSpeedLimit = shootOnMoveSpeedLimit;
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

        m_headingPID.reset();

        // Cache alliance and goal once — these don't change mid-match.
        m_alliance = DriverStation.getAlliance().orElseGet(() -> closestAlliance(
            m_drivetrain.getState().Pose.getTranslation()
        ));
        m_goalPos  = (m_alliance == Alliance.Red) ? kShooter.kRedGoal : kShooter.kBlueGoal;

        // Set shooter distance once from the starting position.
        Translation2d initPos = m_drivetrain.getState().Pose.getTranslation();
        m_shooter.setShootingDistance(initPos.getDistance(m_goalPos));
    }

    /**
     * Returns whichever alliance's hub the robot is currently closer to.
     */
    private static Alliance closestAlliance(Translation2d robotPos) {
        double distRed  = robotPos.getDistance(kShooter.kRedGoal);
        double distBlue = robotPos.getDistance(kShooter.kBlueGoal);
        return (distRed <= distBlue) ? Alliance.Red : Alliance.Blue;
    }

    /**
     * Returns true if the robot is within its alliance's shooting boundary
     * AND the alliance hub is currently active.
     *
     * Called every execute() tick in teleop so the driver can hold the shoot
     * button before crossing into the alliance zone — the drivetrain will keep
     * aiming and the shooter will keep spinning up, but feeding is suppressed
     * until both conditions clear.
     */
    private boolean isClearToFeed(Translation2d robotPos) {
        double x = robotPos.getX();
        boolean inBoundary = (m_alliance == Alliance.Red)
            ? x >= kShooter.redXBoundary
            : x <= kShooter.blueXBoundary;
        return inBoundary && !isHubInactive(m_alliance);
    }

    @Override
    public void execute() {
        // ── 1. Read and clamp joystick inputs ─────────────────────────────────
        // Shoot-on-the-move is teleop-only. Auto always holds position — the
        // path planner has already placed the robot at the shoot point and any
        // joystick input would interfere with the fixed pose.
        double vx = 0.0;
        double vy = 0.0;
        if (!m_isAuto) {
            vx = m_vxSupplier.getAsDouble();
            vy = m_vySupplier.getAsDouble();

            // Clamp to speed limit while preserving the direction vector.
            double speed = Math.hypot(vx, vy);
            if (speed > m_shootOnMoveSpeedLimit && speed > 1e-6) {
                double scale = m_shootOnMoveSpeedLimit / speed;
                vx *= scale;
                vy *= scale;
            }
        }

        // ── 2. Aim at goal using CURRENT pose ─────────────────────────────────
        Translation2d currentPos = m_drivetrain.getState().Pose.getTranslation();

        // Vector from robot to hub
        double dx = m_goalPos.getX() - currentPos.getX();
        double dy = m_goalPos.getY() - currentPos.getY();
        double distToGoal = Math.hypot(dx, dy);

        double targetAngle = Math.atan2(dy, dx);

        // ── 3. Orbital feedforward ─────────────────────────────────────────────
        // As the robot moves, the required facing angle changes. The heading PID
        // alone lags behind, especially when orbiting the hub.
        //
        // The rate of change of the robot→hub angle equals the tangential
        // component of the robot's velocity divided by the distance to the hub:
        //
        //   dθ/dt  =  v_tangential / r
        //
        // Tangential velocity is the component of [vx, vy] perpendicular to the
        // unit vector pointing from robot to hub:
        //
        //   unit_to_hub = (dx/r, dy/r)
        //   tangential  = cross product (2D): (vx * dy/r  -  vy * dx/r)
        //
        // Positive result → angle increasing (CCW orbit) → feedforward is positive.
        // We negate because "facing the hub" means the robot must rotate opposite
        // to how the hub angle sweeps: if the hub angle increases CCW, the robot
        // must rotate CW to keep facing it.
        double orbitalFeedforwardRadPerSec = 0.0;
        if (distToGoal > 0.1) {  // guard against division by near-zero
            double unitX = dx / distToGoal;
            double unitY = dy / distToGoal;
            // tangential velocity = cross(v, unitToHub) in 2D = vx*unitY - vy*unitX
            double vTangential = vx * unitY - vy * unitX;
            // dθ/dt for the hub angle; negate so robot counter-tracks
            orbitalFeedforwardRadPerSec = -(vTangential / distToGoal);
        }

        // Wrap-safe angle error for atAngle check
        boolean atAngle = Math.abs(
            m_drivetrain.getState().Pose.getRotation()
                .minus(Rotation2d.fromRadians(targetAngle))
                .getRadians()
        ) < kShooter.angleTolerance_Rads;

        // ── 4. Apply drivetrain request ────────────────────────────────────────
        // Brake only when there is no requested translation and heading is settled.
        // In auto vx/vy are always 0.0, so this naturally brakes once aimed.
        boolean fullyStationary = Math.hypot(vx, vy) < 1e-6 && atAngle;

        if (fullyStationary) {
            m_drivetrain.setControl(m_brake);
        } else {
            // PID output (rad/s) drives heading toward targetAngle.
            // Orbital feedforward pre-leads the target when orbiting the hub.
            double currentHeading = m_drivetrain.getState().Pose.getRotation().getRadians();
            double pidOutput = m_headingPID.calculate(currentHeading, targetAngle);
            double rotationalRate = pidOutput + orbitalFeedforwardRadPerSec;

            m_drivetrain.setControl(
                m_fieldCentric
                    .withVelocityX(vx)
                    .withVelocityY(vy)
                    .withRotationalRate(rotationalRate)
            );
        }

        // Update shooting distance continuously while moving so the shooter
        // adjusts its speed/angle for the current range to the hub.
        m_shooter.setShootingDistance(distToGoal);

        // ── 5. Feed once aimed, up to speed, and clear to shoot ───────────────
        // In teleop, boundary and hub-active checks are evaluated every tick so
        // the driver can hold the button before crossing into the alliance zone.
        // The drivetrain and shooter keep running during the wait; only feeding
        // is gated. Auto bypasses the positional/shift check entirely.
        boolean clearToFeed = m_isAuto || isClearToFeed(currentPos);

        if (clearToFeed && m_shooter.atTargetRPM() && atAngle) {
            m_feeder.feed();
        } else {
            m_feeder.stop();
        }

        // ── 6. Empty-hopper detection (auto only) ─────────────────────────────
        if (m_isAuto) {
            double elapsed = m_commandTimer.get();
            boolean inDetectionWindow = elapsed >= (m_expectedShootTimeSecs - kShooter.kEarlyWindowSecs);
            m_emptySettleTime = inDetectionWindow
                ? kShooter.kEmptySettleTime_InWindow
                : kShooter.kEmptySettleTime_OutWindow;

            if (m_shooter.atTargetRPM()) {
                if (m_shooter.getStatorCurrent() > kShooter.kShotCurrentThreshold) {
                    m_shotSampleCount++;
                } else {
                    m_shotSampleCount = 0;
                }
            }

            boolean shotConfirmed = m_shotSampleCount >= kShooter.kShotSampleCount;

            if (shotConfirmed) {
                m_shotEverDetected  = true;
                m_shotSampleCount   = 0;
                m_emptyTimer.reset();
                m_emptyTimerRunning = false;
            } else if (m_shotEverDetected) {
                if (!m_emptyTimerRunning) {
                    m_emptyTimer.restart();
                    m_emptyTimerRunning = true;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
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