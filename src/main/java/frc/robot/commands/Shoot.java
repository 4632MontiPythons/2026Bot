package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AimingSolver;
import frc.robot.util.HubSchedule;
import java.util.function.DoubleSupplier;

/**
 * Aims at the goal, spins up the shooter, and feeds game pieces.
 *
 * Teleop: runs until interrupted (button released), or self-terminates when
 * the hub is imminently going inactive.
 *
 * Auto: self-terminates after a fixed timeout (expectedShootTimeSecs)
 */
public class Shoot extends Command {

    // ── Dependencies ──────────────────────────────────────────────────────────

    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final boolean m_isAuto;
    private boolean m_isBraking = false;

    private final DoubleSupplier m_vxSupplier;
    private final DoubleSupplier m_vySupplier;

    /** Expected duration (seconds) to drain the full hopper. Auto only. */
    private final double m_expectedShootTimeSecs;

    // ── Runtime state ─────────────────────────────────────────────────────────

    private final Timer m_commandTimer = new Timer();

    private final SwerveRequest.FieldCentric m_fieldCentric =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    // TUNE
    private final edu.wpi.first.math.controller.PIDController m_headingPID =
        new edu.wpi.first.math.controller.PIDController(5.0, 0.0, 0.1);

    {
        m_headingPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    private Alliance m_alliance;
    private Translation2d m_goalPos;

    // ── Constructors ──────────────────────────────────────────────────────────

    /** Teleop constructor. */
    public Shoot(
        Shooter shooter,
        Feeder feeder,
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier
    ) {
        this(shooter, feeder, drivetrain, vxSupplier, vySupplier, false, 0.0);
    }

    /** Full constructor — teleop and auto. */
    public Shoot(
        Shooter shooter,
        Feeder feeder,
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier,
        boolean isAuto,
        double expectedShootTimeSecs
    ) {
        m_shooter               = shooter;
        m_feeder                = feeder;
        m_drivetrain            = drivetrain;
        m_vxSupplier            = vxSupplier;
        m_vySupplier            = vySupplier;
        m_isAuto                = isAuto;
        m_expectedShootTimeSecs = expectedShootTimeSecs;

        addRequirements(shooter, feeder, drivetrain);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_commandTimer.restart();
        m_headingPID.reset();

        m_alliance = DriverStation.getAlliance().orElseGet(() ->
            closestAlliance(m_drivetrain.getState().Pose.getTranslation()));
        m_goalPos = (m_alliance == Alliance.Red) ? kShooter.kRedHub : kShooter.kBlueHub;

        Translation2d initPos = m_drivetrain.getState().Pose.getTranslation();
        m_shooter.setShootingDistance(initPos.getDistance(m_goalPos));
    }

    @Override
    public void execute() {
        double vx = m_drivetrain.getState().Speeds.vxMetersPerSecond;
        double vy = m_drivetrain.getState().Speeds.vyMetersPerSecond;

        Translation2d currentPos = m_drivetrain.getState().Pose.getTranslation();
        AimingSolver.Solution aim = AimingSolver.solve(currentPos, m_goalPos, vx, vy);

        double targetAngle   = aim.aimAngle;
        double shootDistance = aim.shootDistance;

        // Orbital feedforward
        double orbitalFeedforwardRadPerSec = 0.0;
        if (shootDistance > 0.1) {
            double unitX = Math.cos(targetAngle);
            double unitY = Math.sin(targetAngle);
            double vTangential = vx * unitY - vy * unitX;
            orbitalFeedforwardRadPerSec = -(vTangential / shootDistance);
        }

        double angleError = Math.abs(m_drivetrain.getState().Pose.getRotation()
                    .minus(Rotation2d.fromRadians(targetAngle)).getRadians());
        boolean atAngle = angleError < kShooter.angleTolerance_Rads;
        boolean joystickStationary = Math.hypot(m_vxSupplier.getAsDouble(), m_vySupplier.getAsDouble()) < 1e-6;

        if (joystickStationary && angleError < kShooter.angleTolerance_Rads) {
            m_isBraking = true;
        } else if (!joystickStationary || angleError > kShooter.angleTolerance_Rads * 2) {
            m_isBraking = false;
        }

        if (m_isBraking) {
            m_drivetrain.setControl(m_brake);
        } else {
            double currentHeading = m_drivetrain.getState().Pose.getRotation().getRadians();
            double pidOutput      = m_headingPID.calculate(currentHeading, targetAngle);
            double rotationalRate = pidOutput + orbitalFeedforwardRadPerSec;

            m_drivetrain.setControl(
                m_fieldCentric
                    .withVelocityX(m_vxSupplier.getAsDouble())
                    .withVelocityY(m_vySupplier.getAsDouble())
                    .withRotationalRate(rotationalRate)
            );
        }

        m_shooter.setShootingDistance(shootDistance);

        boolean clearToFeed = m_isAuto || isClearToFeed(currentPos);

        if (clearToFeed && m_shooter.atTargetRPM() && atAngle) {
            m_feeder.feed();
        } else {
            m_feeder.stop();
        }

        // Dashboard
        String status;
        if (!clearToFeed) {
            status = HubSchedule.isHubInactive(m_alliance) ? "Hub Inactive" : "Out of Bounds";
        } else if (!m_shooter.atTargetRPM()) {
            status = "RPM too low";
        } else if (!atAngle) {
            status = "Adjusting Angle";
        } else {
            status = "Firing";
        }
        SmartDashboard.putString("Shooter/Status", status);
        SmartDashboard.putNumber("Shooter/AngleError", Math.toDegrees(angleError));
        SmartDashboard.putNumber("Shooter/ElapsedTime", m_commandTimer.get());
    }

    @Override
    public boolean isFinished() {
        if (!m_isAuto) {
            return HubSchedule.isHubImminentlyInactive(m_alliance, kShooter.kTimeToScore);
        }
        return m_commandTimer.get() >= (m_expectedShootTimeSecs);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private static Alliance closestAlliance(Translation2d robotPos) {
        return (robotPos.getDistance(kShooter.kRedHub) <= robotPos.getDistance(kShooter.kBlueHub))
            ? Alliance.Red
            : Alliance.Blue;
    }

    private boolean isClearToFeed(Translation2d robotPos) {
        double x = robotPos.getX();
        boolean inBoundary = (m_alliance == Alliance.Red)
            ? x >= kShooter.redXBoundary
            : x <= kShooter.blueXBoundary;
        return inBoundary && !HubSchedule.isHubInactive(m_alliance);
    }
}