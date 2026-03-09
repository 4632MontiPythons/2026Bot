package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter;
import frc.robot.MatchInfo;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

/**
 * Shoot — aims at the goal, spins up the shooter, and feeds game pieces.
 *
 * In teleop mode (isAuto = false): runs until interrupted (button released).
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

    private final PIDController m_thetaController = new PIDController(5.0, 0, 0);
    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric m_fieldCentric = new SwerveRequest.FieldCentric();

    // ── Constructors ──────────────────────────────────────────────────────────

    /**
     * Teleop constructor — runs until interrupted.
     */
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

        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_thetaController.setTolerance(kShooter.angleTolerance_Rads, kShooter.angleTolerance_RadsPerSec);
        addRequirements(shooter, feeder, drivetrain);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_commandTimer.restart();
        m_emptyTimer.reset();
        m_emptyTimerRunning = false;
        m_thetaController.reset();
    }

    @Override
    public void execute() {
        // ── 1. Aim at goal ────────────────────────────────────────────────────
        Translation2d robotPos = m_drivetrain.getState().Pose.getTranslation();
        Alliance alliance = MatchInfo.getInstance().getOwnAlliance().orElse(Alliance.Blue);
        Translation2d target = (alliance == Alliance.Red) ? kShooter.kRedGoal : kShooter.kBlueGoal;

        double targetAngle = Math.atan2(
            target.getY() - robotPos.getY(),
            target.getX() - robotPos.getX()
        );

        double rotationOutput = m_thetaController.calculate(
            m_drivetrain.getState().Pose.getRotation().getRadians(),
            targetAngle
        );

        if (m_thetaController.atSetpoint()) {
            m_drivetrain.applyRequest(() -> m_brake);
        } else {
            m_drivetrain.setControl(
                m_fieldCentric
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotationOutput)
            );
        }

        // ── 2. Set shooter speed  ────────────────────────────────────────────────
        m_shooter.setShootingDistance(robotPos.getDistance(target));

        // ── 3. Feed once aimed and up to speed ────────────────────────────────
        if (m_shooter.atTargetRPM() && m_thetaController.atSetpoint()) {
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
        if (!m_isAuto) return false; // in teleop only finish when button released

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