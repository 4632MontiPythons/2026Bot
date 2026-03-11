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
 * In teleop mode (isAuto = false): runs until interrupted (button released).
 * Aborts early if the robot is outside its alliance shooting boundary or
 * it is not the robot's active shift (teleop only).
 *
 * In auto mode (isAuto = true): self-terminates when the hopper is empty
 * or a hard timeout fires. Empty detection is gated by a time window to
 * prevent false positives early in the run.
 *
 *   Empty detection window opens at:  (expectedShootTimeSecs - kEarlyWindowSecs)
 *   Hard timeout fires at:            (expectedShootTimeSecs + kLateWindowSecs)
 */
public class Shoot extends Command {

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

    private final SwerveRequest.FieldCentricFacingAngle m_fieldCentricFacingAngle =
        new SwerveRequest.FieldCentricFacingAngle()
            .withHeadingPID(5.0, 0, 0);

    /** Set in initialize(); used for boundary check throughout the command. */
    private Translation2d m_initRobotPos;

    /** Set in initialize(); false triggers early exit in teleop. */
    private boolean m_preflightPassed = false;

    // ── Constructors ──────────────────────────────────────────────────────────

    /** Teleop constructor — runs until interrupted. */
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
        m_initRobotPos = m_drivetrain.getState().Pose.getTranslation();
        m_preflightPassed = m_isAuto || checkPreflight(m_initRobotPos);
    }

    /**
     * Validates boundary and shift conditions at the start of a teleop shoot.
     * Posts a Elastic error notification and returns false if any check fails.
     */
    private boolean checkPreflight(Translation2d robotPos) {
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        double x = robotPos.getX();

        // Boundary check
        boolean inBoundary = (alliance == Alliance.Red)
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
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData != null && !gameData.isEmpty()) {
            char ownAllianceChar = (alliance == Alliance.Red) ? 'R' : 'B';
            boolean ownAllianceInactive = (ownAllianceChar == gameData.charAt(0));
            if (ownAllianceInactive) {
                Elastic.sendNotification(new Elastic.Notification()
                    .withLevel(Elastic.NotificationLevel.ERROR)
                    .withTitle("Shoot Blocked")
                    .withDescription("Cannot shoot during inactive shift.")
                );
                return false;
            }
        }

        return true;
    }

    @Override
    public void execute() {
        if (!m_preflightPassed) return;

        // ── 1. Aim at goal ────────────────────────────────────────────────────
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Translation2d target = (alliance == Alliance.Red) ? kShooter.kRedGoal : kShooter.kBlueGoal;

        double targetAngle = Math.atan2(
            target.getY() - m_initRobotPos.getY(),
            target.getX() - m_initRobotPos.getX()
        );

        m_drivetrain.setControl(
            m_fieldCentricFacingAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(Rotation2d.fromRadians(targetAngle))
        );

        // ── 2. Set shooter speed ──────────────────────────────────────────────
        m_shooter.setShootingDistance(m_initRobotPos.getDistance(target));

        // ── 3. Feed once aimed and up to speed ────────────────────────────────
        boolean atAngle = Math.abs(
            m_drivetrain.getState().Pose.getRotation().getRadians() - targetAngle
        ) < kShooter.angleTolerance_Rads;

        if (m_shooter.atTargetRPM() && atAngle) {
            m_feeder.feed();
        } else {
            m_feeder.pause();
        }

        // ── 4. Empty-hopper detection (auto only, gated by time window) ────────
        if (m_isAuto) {
            double elapsed = m_commandTimer.get();
            boolean inDetectionWindow = elapsed >= (m_expectedShootTimeSecs - kShooter.kEarlyWindowSecs);

            if (inDetectionWindow) {
                if (m_feeder.getOutputCurrent() < kShooter.kEmptyCurrentThreshold) {
                    if (!m_emptyTimerRunning) {
                        m_emptyTimer.restart();
                        m_emptyTimerRunning = true;
                    }
                } else {
                    m_emptyTimer.reset();
                    m_emptyTimerRunning = false;
                }
            } else {
                m_emptyTimer.reset();
                m_emptyTimerRunning = false;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (!m_preflightPassed) return true;
        if (!m_isAuto) return false;

        boolean hopperEmpty = m_emptyTimerRunning && m_emptyTimer.hasElapsed(kShooter.kEmptySettleTime);
        boolean timedOut    = m_commandTimer.get() >= (m_expectedShootTimeSecs + kShooter.kLateWindowSecs);

        return hopperEmpty || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
    }
}