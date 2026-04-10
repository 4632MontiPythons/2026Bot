package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import frc.robot.Constants.kShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class Funnel extends Command {

    private final Shooter m_shooter;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Feeder m_feeder;
    private final DoubleSupplier m_vxSupplier;
    private final DoubleSupplier m_vySupplier;
    private final boolean m_auto;

    private Alliance m_alliance;
    private Translation2d m_activeTargetPoint;
    private double m_targetAngleRads;

    private final PIDController m_headingPID = new PIDController(5.0, 0.0, 0.1);

    private final SwerveRequest.FieldCentric m_fieldCentric = 
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(0.45);

    public Funnel(
        Shooter shooter,
        CommandSwerveDrivetrain drivetrain,
        Feeder feeder,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier,
        boolean auto
    ) {
        m_shooter    = shooter;
        m_drivetrain = drivetrain;
        m_feeder     = feeder;
        m_vxSupplier = vxSupplier;
        m_vySupplier = vySupplier;
        m_auto       = auto;

        // ── Conditional Requirements ──
        if (m_auto) {
            addRequirements(shooter, feeder); // Drivetrain NOT required in auto
        } else {
            addRequirements(shooter, feeder, drivetrain);
        }
    }

    @Override
    public void initialize() {
        m_headingPID.reset();
        m_headingPID.enableContinuousInput(-Math.PI, Math.PI);

        Translation2d robotPos = m_drivetrain.getState().Pose.getTranslation();
        m_alliance = DriverStation.getAlliance().orElseGet(() -> (robotPos.getX() > 8.27 ? Alliance.Red : Alliance.Blue));

        Translation2d pointLeft = (m_alliance == Alliance.Red) ? kShooter.kRedFunnelLeft : kShooter.kBlueFunnelLeft;
        Translation2d pointRight = (m_alliance == Alliance.Red) ? kShooter.kRedFunnelRight : kShooter.kBlueFunnelRight;

        m_activeTargetPoint = (robotPos.getDistance(pointLeft) < robotPos.getDistance(pointRight)) ? pointLeft : pointRight;
    }

    @Override
    public void execute() {
        var state = m_drivetrain.getState();
        Translation2d robotPos = state.Pose.getTranslation();
        Rotation2d currentRotation = state.Pose.getRotation();

        // 1. Aiming Logic
        m_targetAngleRads = Math.atan2(
            m_activeTargetPoint.getY() - robotPos.getY(), 
            m_activeTargetPoint.getX() - robotPos.getX()
        );

        double angleError = Math.abs(( currentRotation.minus(Rotation2d.fromRadians(m_targetAngleRads)) ).getRadians());
        boolean atAngle = angleError < kShooter.angleTolerance_Rads*3;

        m_shooter.setShootingDistance(3.0); // TUNE

        // 3. Drivetrain Control (Only if NOT in auto)
        if (!m_auto) {
            double pidOutput = m_headingPID.calculate(currentRotation.getRadians(), m_targetAngleRads);
            m_drivetrain.setControl(
                m_fieldCentric
                    .withVelocityX(m_vxSupplier.getAsDouble())
                    .withVelocityY(m_vySupplier.getAsDouble())
                    .withRotationalRate(pidOutput)
            );
        }

        // 4. Feed Gating
        boolean clearToFeed = m_auto || isClearToFeed(robotPos);
        boolean rpmReady = m_shooter.atTargetRPM(3 * kShooter.rpmTolerance);

        // In auto, we ignore 'atAngle' check if we assume the path is handling heading
        if (clearToFeed && rpmReady && (atAngle || m_auto)) {
            m_feeder.feed();
        }

        SmartDashboard.putNumber("Shooter/AngleError", Math.toDegrees(angleError));
    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {}

    private boolean isClearToFeed(Translation2d robotPos) {
        double x = robotPos.getX();
        return x >= kShooter.blueXBoundary && x <= kShooter.redXBoundary;
    }
}