package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter;
import frc.robot.MatchInfo;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class TeleopShoot extends Command {
    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final CommandSwerveDrivetrain m_drivetrain;
    // Inside your TeleopShoot class
    private final DoubleSupplier m_translationX;
    private final DoubleSupplier m_translationY;
    private final PIDController m_thetaController = new PIDController(5.0, 0, 0);

    public TeleopShoot(
        Shooter shooter, 
        Feeder feeder, 
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier translationX,
        DoubleSupplier translationY
    ) {
        m_shooter = shooter;
        m_feeder = feeder;
        m_drivetrain = drivetrain;
        m_translationX = translationX;
        m_translationY = translationY;

        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(shooter, feeder, drivetrain);
    }

    @Override
    public void execute() {
        Translation2d robotPos = m_drivetrain.getState().Pose.getTranslation();
        Alliance alliance = MatchInfo.getInstance().getOwnAlliance().orElse(Alliance.Blue);
        Translation2d target = (alliance == Alliance.Red) ? kShooter.kRedGoal : kShooter.kBlueGoal;

        //calculate heading to goal
        double targetAngle = Math.atan2(
            target.getY() - robotPos.getY(), 
            target.getX() - robotPos.getX()
        );

        // the driver's X/Y but the PID's Rotation
        double rotationOutput = m_thetaController.calculate(
            m_drivetrain.getState().Pose.getRotation().getRadians(), 
            targetAngle
        );

        m_drivetrain.setControl(
            new SwerveRequest.FieldCentric()
                .withVelocityX(m_translationX.getAsDouble())
                .withVelocityY(m_translationY.getAsDouble())
                .withRotationalRate(rotationOutput)
        );

        //run shooter at speed based on distance to target
        m_shooter.setShootingDistance(robotPos.getDistance(target));
        if(m_shooter.atTargetRPM()) {
            m_feeder.feed();
        } else {
            m_feeder.stop();
        }
    }
    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
    }
}