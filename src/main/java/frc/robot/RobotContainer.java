// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.OI;
import frc.robot.commands.Shoot;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.Constants.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
        private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
        private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        private final SlewRateLimiter xSlewLimiter = new SlewRateLimiter(OI.slewRate);
        private final SlewRateLimiter ySlewLimiter = new SlewRateLimiter(OI.slewRate);

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * OI.deadband)
                        .withRotationalDeadband(MaxAngularRate * OI.deadband) // Add a deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SendableChooser<Command> autoChooser;
        // private final Telemetry logger = new Telemetry();

        private final CommandXboxController xboxController = new CommandXboxController(OI.driverControllerPort);

        public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
                TunerConstants.DrivetrainConstants,
                0, // odometry update frequency (0 = use default)
                VecBuilder.fill(Drive.odometryXYStdDevs, Drive.odometryXYStdDevs, Drive.odometryYawStdDev),
                VecBuilder.fill(999, 999, 999), // this is the *default* vision std dev. These values are never used because we always dynamically set
                TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft,TunerConstants.BackRight
                );
        // private final Shooter shooter = new Shooter();
        // private final Feeder feeder = new Feeder();
        // private final Intake intake = new Intake();


        public RobotContainer() {
                //register named commands for pathplanner
                NamedCommands.registerCommand(
                        "WheelRadiusCharacterization", 
                        new WheelRadiusCharacterization(drivetrain)
                );
                // NamedCommands.registerCommand(
                //         "Deploy Intake", 
                //         Commands.runOnce(() -> intake.deploy(), intake)
                // );
                // NamedCommands.registerCommand(
                //         "Retract Intake", 
                //         Commands.runOnce(() -> intake.deploy(), intake)
                // );
                NamedCommands.registerCommand(
                        "ShootFullHopper", 
                        new Shoot(null,null,null,true,10)
                );
                // NamedCommands.registerCommand(
                //         "Run Intake", 
                //         Commands.run(() -> intake.runIntake(), intake)
                //                 .finallyDo(() -> intake.stopIntake())
                // );
                // NamedCommands.registerCommand(
                //         "Toggle Intake On", 
                //         Commands.run(() -> intake.runIntake(), intake)
                // );
                // NamedCommands.registerCommand(
                //         "Toggle Intake Off", 
                //         Commands.run(() -> intake.stopIntake(), intake)
                // );
                // NamedCommands.registerCommand(
                //         "Spin Up Shooter", 
                //         Commands.run(() -> shooter.setShootingDistance(3.00)
                // );
                NamedCommands.registerCommand(
                        "ShootDepot", 
                        new Shoot(null,null,null,true,5)
                );
                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureBindings() {

                //default command; runs when nothing else is using drivetrain
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(xSlewLimiter.calculate(-xboxController.getLeftY())
                                                                * MaxSpeed)
                                                .withVelocityY(ySlewLimiter.calculate(-xboxController.getLeftX())
                                                                * MaxSpeed)
                                                .withRotationalRate(-xboxController.getRightX()
                                                                * MaxAngularRate)));

                // Idle while the robot is disabled
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                xboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // xboxController.b().whileTrue(drivetrain.applyRequest(() -> point
                //                 .withModuleDirection(new Rotation2d(-xboxController.getLeftY(),-xboxController.getLeftX()))));

                // reset the field-centric heading on left bumper press(LB)
                xboxController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // shoot fuel while held
                xboxController.rightTrigger().whileTrue(new Shoot(null, null, drivetrain));

                // xboxController.rightBumper().whileTrue(Commands.run(() -> shooter.setShootingDistance(3.00)))
                //         .finallyDo(shooter.stop);

                // the following bindings only do anything if drive.comp is false(not in a
                // competition setting). that boolean has to be manually set
                if (!Drive.comp) {
                        // Run SysId routines when holding back/start and X/Y.
                        // Note that each routine should be run exactly once in a single log.
                        xboxController.back().and(xboxController.y())
                                        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                        xboxController.back().and(xboxController.x())
                                        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                        xboxController.start().and(xboxController.y())
                                        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                        xboxController.start().and(xboxController.x())
                                        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                        // For testing purposes only: reset position to in front of the center of the
                        // red alliance hub, facing hub
                        xboxController.rightBumper().onTrue(
                                        new InstantCommand(() -> drivetrain.resetPose(
                                                        new Pose2d((492.88 + 13.5) * 0.0254, (158.84) * 0.0254,
                                                                        Rotation2d.fromDegrees(180)))));
                        //uncomment when shooter ready
                        // xboxController.povLeft().and(xboxController.y())
                        //         .whileTrue(shooter.sysIdDynamic(Direction.kForward));
                        // xboxController.povLeft().and(xboxController.x())
                        //         .whileTrue(shooter.sysIdDynamic(Direction.kReverse));
                        // xboxController.povRight().and(xboxController.y())
                        //         .whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
                        // xboxController.povRight().and(xboxController.x())
                        //         .whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
                }
                // if(Drive.log) drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
