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
        private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private final double MaxAngularRate = Drive.maxAngularRateRadPerSec;

        private final SlewRateLimiter xSlewLimiter = new SlewRateLimiter(OI.slewRate);
        private final SlewRateLimiter ySlewLimiter = new SlewRateLimiter(OI.slewRate);

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * OI.deadband)
                        .withRotationalDeadband(MaxAngularRate * OI.deadband)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SendableChooser<Command> autoChooser;

        private final CommandXboxController xboxController = new CommandXboxController(OI.driverControllerPort);

        public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
                TunerConstants.DrivetrainConstants,
                0,
                VecBuilder.fill(Drive.odometryXYStdDevs, Drive.odometryXYStdDevs, Drive.odometryYawStdDev),
                VecBuilder.fill(999, 999, 999),
                TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight
                );
        private final Shooter shooter = new Shooter();
        private final Intake intake = new Intake();
        private final Feeder feeder = new Feeder();


        public RobotContainer() {
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
                //         Commands.runOnce(() -> intake.retract(), intake)
                // );
                // NamedCommands.registerCommand(
                //         "ShootFullHopper",
                //         new Shoot(shooter, feeder, drivetrain,
                //                 () -> 0.0, () -> 0.0,      // stationary in auto
                //                 MaxSpeed * kShootOnMoveSpeedFraction,
                //                 true, 11)
                // );
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
                //         Commands.run(() -> shooter.setShootingDistance(3.00), shooter)
                // );
                // NamedCommands.registerCommand(
                //         "ShootDepot",
                //         new Shoot(shooter, feeder, drivetrain,
                //                 () -> 0.0, () -> 0.0,
                //                 MaxSpeed * kShootOnMoveSpeedFraction,
                //                 true, 6)
                // );
                
                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureBindings() {

                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(xSlewLimiter.calculate(-xboxController.getLeftY())
                                                                * MaxSpeed)
                                                .withVelocityY(ySlewLimiter.calculate(-xboxController.getLeftX())
                                                                * MaxSpeed)
                                                .withRotationalRate(-xboxController.getRightX()
                                                                * MaxAngularRate)));
                

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                xboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));

                // reset the field-centric heading on left bumper press (LB)
                xboxController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                xboxController.b().onTrue(Commands.run(() -> intake.deploy(), intake));
                xboxController.y().onTrue(Commands.run(() -> intake.retract(), intake));
                xboxController.rightBumper().whileTrue(Commands.run(() -> intake.runIntake(), intake));
                xboxController.x().onTrue(Commands.run(() -> shooter.test(), shooter));
                xboxController.leftTrigger().whileTrue(Commands.run(() -> feeder.test(), feeder));
                xboxController.rightTrigger().whileTrue(Commands.run(() -> intake.reverseIntake(), intake));

                // ── Shoot on the move (right trigger held) ────────────────────────────
                // Joystick inputs are read live each tick inside the command.
                // Speed is capped at kShootOnMoveSpeedFraction of MaxSpeed.
                // Uncomment when shooter/feeder are wired up.
                //
                // xboxController.rightTrigger().whileTrue(new Shoot(
                //         shooter, feeder, drivetrain,
                //         () -> xSlewLimiter.calculate(-xboxController.getLeftY()) * MaxSpeed,
                //         () -> ySlewLimiter.calculate(-xboxController.getLeftX()) * MaxSpeed,
                //         MaxSpeed * kShootOnMoveSpeedFraction
                // ));

                if (!Drive.comp) {
                        xboxController.back().and(xboxController.y())
                                        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                        xboxController.back().and(xboxController.x())
                                        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                        xboxController.start().and(xboxController.y())
                                        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                        xboxController.start().and(xboxController.x())
                                        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                        xboxController.rightBumper().onTrue(
                                        new InstantCommand(() -> drivetrain.resetPose(
                                                        new Pose2d((492.88 + 13.5) * 0.0254, (158.84) * 0.0254,
                                                                        Rotation2d.fromDegrees(180)))));
                }
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}