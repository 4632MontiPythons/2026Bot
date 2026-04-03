package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.kShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AimingSolver;
import frc.robot.util.HubSchedule;

/**
 * Aims at the goal, spins up the shooter, and feeds game pieces.
 */
public class Shoot extends Command {

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final boolean m_pathplannerControlsDrive;
    private final BooleanSupplier m_brakeButton; // Fixed: using consistent import
    private final DoubleSupplier m_vxSupplier;
    private final DoubleSupplier m_vySupplier;
    private final boolean m_isAuto;
    private final double m_expectedShootTimeSecs;

    // ── Runtime state ─────────────────────────────────────────────────────────
    private final Timer m_commandTimer = new Timer();

    private final SwerveRequest.FieldCentric m_fieldCentric = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(0.458);
    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    private final PIDController m_headingPID = new PIDController(5.0, 0.0, 0.1);

    private Alliance m_alliance;
    private Translation2d m_goalPos;

    private boolean m_hubWasActive = false;
    private boolean m_shouldFinish = false;

    // ── Constructors ──────────────────────────────────────────────────────────

    /** Teleop constructor. */
    public Shoot(Shooter shooter, Feeder feeder, CommandSwerveDrivetrain drivetrain, 
                DoubleSupplier vxSupplier, DoubleSupplier vySupplier, BooleanSupplier brakeButton) {
        this(shooter, feeder, drivetrain, vxSupplier, vySupplier, false, 0.0, false, brakeButton);
    }

    /** Full constructor — teleop and auto. */
    public Shoot(Shooter shooter, Feeder feeder, CommandSwerveDrivetrain drivetrain, 
                DoubleSupplier vxSupplier, DoubleSupplier vySupplier, 
                boolean isAuto, double expectedShootTimeSecs, boolean pathplannerControlsDrive, BooleanSupplier brakeButton) {
        m_shooter = shooter;
        m_feeder = feeder;
        m_drivetrain = drivetrain;
        m_vxSupplier = vxSupplier;
        m_vySupplier = vySupplier;
        m_isAuto = isAuto;
        m_expectedShootTimeSecs = expectedShootTimeSecs;
        m_pathplannerControlsDrive = pathplannerControlsDrive;
        m_brakeButton = brakeButton; // Fixed: Added assignment

        m_headingPID.enableContinuousInput(-Math.PI, Math.PI);

        if (pathplannerControlsDrive) {
            addRequirements(shooter, feeder);
        } else {
            addRequirements(shooter, feeder, drivetrain);
        }
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_commandTimer.restart();
        m_headingPID.reset();

        m_shouldFinish = false;
        m_hubWasActive = false;

        Translation2d initPos = m_drivetrain.getState().Pose.getTranslation();
        m_alliance = DriverStation.getAlliance().orElseGet(() -> closestAlliance(initPos));
        m_goalPos = (m_alliance == Alliance.Red) ? kShooter.kRedHub : kShooter.kBlueHub;

        m_shooter.setShootingDistance(initPos.getDistance(m_goalPos));
    }

    @Override
    public void execute() {
        var state = m_drivetrain.getState();
        double vx = state.Speeds.vxMetersPerSecond;
        double vy = state.Speeds.vyMetersPerSecond;
        Translation2d currentPos = state.Pose.getTranslation();
        Rotation2d currentRotation = state.Pose.getRotation();

        AimingSolver.Solution aim = AimingSolver.solve(currentPos, m_goalPos, vx, vy);
        m_shooter.setShootingDistance(aim.shootDistance);
        
        boolean hubShootWindowOpen = HubSchedule.isHubShootWindowOpen(m_alliance);

        double angleError = Math.abs(currentRotation.minus(Rotation2d.fromRadians(aim.aimAngle)).getRadians());
        boolean atAngle = angleError < kShooter.angleTolerance_Rads;

        double joyVx = m_vxSupplier.getAsDouble();
        double joyVy = m_vySupplier.getAsDouble();

        if (!m_pathplannerControlsDrive) {
            // Manual Brake override inside the command
            if (m_brakeButton.getAsBoolean()) {
                m_drivetrain.setControl(m_brake);
            } else {
                double pidOutput = m_headingPID.calculate(currentRotation.getRadians(), aim.aimAngle);
                m_drivetrain.setControl(m_fieldCentric
                    .withVelocityX(joyVx)
                    .withVelocityY(joyVy)
                    .withRotationalRate(pidOutput)
                );
            }
        }

        if (hubShootWindowOpen && m_shooter.atTargetRPM(kShooter.rpmTolerance) && (atAngle || m_pathplannerControlsDrive)) {
            m_feeder.feed();
        } else {
            m_feeder.stop();
        }

        if (m_isAuto) {
            m_shouldFinish = m_commandTimer.hasElapsed(m_expectedShootTimeSecs);
        } else {
            m_shouldFinish = m_hubWasActive && !hubShootWindowOpen;
        }
        m_hubWasActive |= hubShootWindowOpen;

        // Dashboard
        String status = !hubShootWindowOpen ? "Hub Inactive" : 
                        !m_shooter.atTargetRPM(kShooter.rpmTolerance) ? "RPM too low" : 
                        !atAngle ? "Adjusting Angle" : "Firing";

        SmartDashboard.putString("Shooter/Status", status);
        SmartDashboard.putNumber("Shooter/AngleError", Math.toDegrees(angleError));
    }

    @Override
    public boolean isFinished() {
        return m_shouldFinish;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
    }

    private static Alliance closestAlliance(Translation2d robotPos) {
        return (robotPos.getDistance(kShooter.kRedHub) <= robotPos.getDistance(kShooter.kBlueHub))
            ? Alliance.Red : Alliance.Blue;
    }
}