package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
 * Shoots while button is held. Orients to correct angle and brakes. 
 * Runs feeder when at correct angle and rpm.
 */
public class TeleopShoot extends Command {
    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final PIDController m_thetaController = new PIDController(5.0, 0, 0);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric();

    public TeleopShoot(
        Shooter shooter,
        Feeder feeder,
        CommandSwerveDrivetrain drivetrain
    ) {
        m_shooter = shooter;
        m_feeder = feeder;
        m_drivetrain = drivetrain;

        m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(shooter, feeder, drivetrain);
    }
    @Override
    public void execute() {
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
            m_drivetrain.applyRequest(() -> brake);
        } else {
            m_drivetrain.setControl(
                fieldCentric
                    .withVelocityX(0)
                    .withVelocityY(0)
                    .withRotationalRate(rotationOutput)
            );
        }

        m_shooter.setShootingDistance(robotPos.getDistance(target));
        if (m_shooter.atTargetRPM() && m_thetaController.atSetpoint()) {
            m_feeder.feed();
        } else {
            m_feeder.pause();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
    }
}