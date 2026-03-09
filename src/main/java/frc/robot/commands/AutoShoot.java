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
 * AutoShoot — drains a certain amount of fuel into the shooter.
 *
 * isFinished check: the feeder output current drops below kEmptyCurrentThreshold
 * and stays there for kEmptySettleTime seconds, indicating nothing is left to feed.
 *
 * Two guard rails prevent early finishing:
 *   1. Empty detection is only considered valid after
 *      (expectedShootTimeSecs - kEarlyWindowSecs) have elapsed, so brief
 *      gaps between game pieces early in the run don't fool us.
 *   2. A hard timeout fires at (expectedShootTimeSecs + kLateWindowSecs)
 *      regardles
 */
public class AutoShoot extends Command {

    // ── Tuning ────────────────────────────────────────────────────────────────

    /**
     * Feeder output current (A) below which we consider the feeder unloaded.
     * Tune this: a game piece being fed should pull well above this value;
     * an empty feeder should idle well below it.
     */
    private static final double kEmptyCurrentThreshold = 3.0;

    /**
     * How long (s) feeder current must stay below kEmptyCurrentThreshold
     * before we declare the hopper empty. Long enough to bridge the gap
     * between consecutive game pieces, short enough to end promptly.
     * Start around 0.3s and tune up if you get false empties mid-hopper.
     */
    private static final double kEmptySettleTime = 0.5;

    /**
     * How many seconds BEFORE expectedShootTimeSecs we begin listening for
     * an empty-hopper signal. Before this window opens, current drops are ignored.
     * Prevents false detection on the brief gap when feeding first starts.
     */
    private static final double kEarlyWindowSecs = 3.0;

    /**
     * How many seconds AFTER expectedShootTimeSecs we keep waiting before
     * giving up and ending the command via hard timeout.
     */
    private static final double kLateWindowSecs = 2.0;

    // ── Dependencies ──────────────────────────────────────────────────────────

    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final CommandSwerveDrivetrain m_drivetrain;

    /**
     * Expected duration (seconds) to drain the full hopper.
     * Used to gate empty detection and set the hard timeout.
     */
    private final double m_expectedShootTimeSecs;

    // ── Runtime state ─────────────────────────────────────────────────────────

    private final Timer m_commandTimer  = new Timer(); // total elapsed since initialize()
    private final Timer m_emptyTimer    = new Timer(); // how long current has been low
    private boolean m_emptyTimerRunning = false;

    private final PIDController m_thetaController = new PIDController(5.0, 0, 0);
    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric m_fieldCentric = new SwerveRequest.FieldCentric();

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param shooter              shooter subsystem
     * @param feeder               feeder subsystem
     * @param drivetrain           swerve drivetrain
     * @param expectedShootTimeSecs estimated seconds to drain the full hopper;
     *                             empty detection is suppressed until
     *                             (expectedShootTimeSecs - kEarlyWindowSecs)
     */
    public AutoShoot(
        Shooter shooter,
        Feeder feeder,
        CommandSwerveDrivetrain drivetrain,
        double expectedShootTimeSecs
    ) {
        m_shooter = shooter;
        m_feeder  = feeder;
        m_drivetrain = drivetrain;
        m_expectedShootTimeSecs = expectedShootTimeSecs;

        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        m_thetaController.setTolerance(kShooter.angleTolerance_Rads,kShooter.angleTolerance_RadsPerSec);
        addRequirements(shooter, feeder, drivetrain);
    }


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

        // ── 2. Spin up shooter ────────────────────────────────────────────────
        m_shooter.setShootingDistance(robotPos.getDistance(target));

        // ── 3. Feed once aimed and up to speed ────────────────────────────────
        if (m_shooter.atTargetRPM() && m_thetaController.atSetpoint()) {
            m_feeder.feed();
        } else {
            m_feeder.pause();
        }

        // ── 4. Empty-hopper detection (gated by time window) ──────────────────
        double elapsed = m_commandTimer.get();
        double windowOpenAt = m_expectedShootTimeSecs - kEarlyWindowSecs;
        boolean inDetectionWindow = elapsed >= windowOpenAt;

        if (inDetectionWindow) {
            boolean feederIdle = m_feeder.getOutputCurrent() < kEmptyCurrentThreshold;

            if (feederIdle) {
                if (!m_emptyTimerRunning) {
                    // Current just dropped — start settle timer
                    m_emptyTimer.restart();
                    m_emptyTimerRunning = true;
                }
                // else: timer already running, let it accumulate
            } else {
                // Current spiked back up — a new piece is feeding; reset settle timer
                m_emptyTimer.reset();
                m_emptyTimerRunning = false;
            }
        } else {
            // Outside detection window — keep settle timer cleared
            m_emptyTimer.reset();
            m_emptyTimerRunning = false;
        }
    }

    @Override
    public boolean isFinished() {
        double elapsed = m_commandTimer.get();

        // hopper empty signal sustained long enough inside the reasonable window
        boolean hopperEmpty = m_emptyTimerRunning
                && m_emptyTimer.hasElapsed(kEmptySettleTime);

        // hard timeout,
        boolean timedOut = elapsed >= (m_expectedShootTimeSecs + kLateWindowSecs);

        return hopperEmpty || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
    }
}