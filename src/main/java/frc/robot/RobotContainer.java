// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.OI;
import frc.robot.commands.Funnel;
import frc.robot.commands.Shoot;
// import frc.robot.commands.SwallowIntake;
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
        // private boolean m_shootActive = false;

        private final SlewRateLimiter xSlewLimiter = new SlewRateLimiter(OI.slewRate);
        private final SlewRateLimiter ySlewLimiter = new SlewRateLimiter(OI.slewRate);

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * OI.deadband)
                        .withRotationalDeadband(MaxAngularRate * OI.deadband)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SendableChooser<Command> autoChooser;

        private final CommandXboxController mainController = new CommandXboxController(OI.driverControllerPort);
        private final CommandXboxController secondaryController = new CommandXboxController(OI.driverControllerPort + 1);

        public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
                TunerConstants.DrivetrainConstants,
                0,
                VecBuilder.fill(Drive.odometryXYStdDevs, Drive.odometryXYStdDevs, Drive.odometryYawStdDev),
                VecBuilder.fill(999, 999, 999), //default standard deviation. always set dynamically. falls back to this if errors i guess
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
                NamedCommands.registerCommand(
                        "Deploy Intake", 
                        Commands.runOnce(() -> intake.deploy(), intake)
                );
                NamedCommands.registerCommand(
                        "Retract Intake", 
                        Commands.runOnce(() -> intake.retract(), intake)
                );
                NamedCommands.registerCommand(
                        "ShootFullHopper",
                        new Shoot(shooter, feeder, drivetrain,
                                () -> 0.0, () -> 0.0,      // stationary in auto
                                true, 8) //TUNE
                );
                NamedCommands.registerCommand( //zoned event in pathplanner
                        "Run Intake", 
                        Commands.run(() -> intake.runIntake(), intake)
                                .finallyDo(() -> intake.stopIntake())
                );
                NamedCommands.registerCommand(
                "Spin Up Shooter", //zoned event in pathplanner
                Commands.run(() -> shooter.warmUp(), shooter)
                        .finallyDo(() -> shooter.stop())
                );
                NamedCommands.registerCommand(
                        "ShootDepot",
                        new Shoot(shooter, feeder, drivetrain,
                                () -> 0.0, () -> 0.0,
                                true, 8) //TUNE
                );
                NamedCommands.registerCommand(
                        "Shoot8",
                        new Shoot(shooter, feeder, drivetrain,
                                () -> 0.0, () -> 0.0,
                                true, 3) //TUNE
                );
                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureBindings() {
                // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> intake.deploy(), intake));
                //we always want to immediately deploy the intake. we do it anyway in the auto, but this is a safekeeping measure


                // ── Default drive command ─────────────────────────────────────────────
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(xSlewLimiter.calculate(-mainController.getLeftY())
                                                                * MaxSpeed)
                                                .withVelocityY(ySlewLimiter.calculate(-mainController.getLeftX())
                                                                * MaxSpeed)
                                                .withRotationalRate(-mainController.getRightX()
                                                                * MaxAngularRate)));

                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // ── Main controller ───────────────────────────────────────────────────
                // Reset field-centric heading (x)
                mainController.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                // Toggle intake deployment
                mainController.leftBumper().onTrue(Commands.runOnce(() -> intake.toggleIntake(), intake));

                // Trigger shootingTrigger = new Trigger(() -> m_shootActive);
                // Shoot on the move (right trigger held)
                // Joystick inputs are read live each tick inside the command.
                // Speed is capped at kShootOnMoveSpeedFraction of MaxSpeed.
                mainController.rightTrigger().whileTrue(new Shoot(
                        shooter, feeder, drivetrain,
                        () -> xSlewLimiter.calculate(-mainController.getLeftY()) * MaxSpeed,
                        () -> ySlewLimiter.calculate(-mainController.getLeftX()) * MaxSpeed)
                        // .beforeStarting(() -> m_shootActive = true)
                        // .finallyDo(interrupted -> m_shootActive = false)
                );
                mainController.rightBumper().whileTrue(new Funnel(
                        shooter, drivetrain, feeder,
                        () -> xSlewLimiter.calculate(-mainController.getLeftY()) * MaxSpeed,
                        () -> ySlewLimiter.calculate(-mainController.getLeftX()) * MaxSpeed
                ));


                // SwallowIntake: intake faces direction of travel (left trigger held)
                // mainController.leftTrigger()
                // .and(shootingTrigger.negate())
                // .whileTrue(new SwallowIntake(
                //         drivetrain, intake,
                //         () -> xSlewLimiter.calculate(-mainController.getLeftY()) * MaxSpeed,
                //         () -> ySlewLimiter.calculate(-mainController.getLeftX()) * MaxSpeed
                // ));
                //when we're already shooting, prioritize that, and run basic intake without heading control
                mainController.leftTrigger()
                // .and(shootingTrigger)
                .whileTrue(Commands.run(() -> intake.runIntake(), intake)
                        .finallyDo(() -> intake.stopIntake()));


                // ── Secondary controller ──────────────────────────────────────────────
                secondaryController.b().whileTrue(drivetrain.applyRequest(() -> brake));
                secondaryController.leftTrigger().whileTrue(Commands.run(() -> intake.runIntake(), intake).finallyDo(() -> intake.stopIntake()));
                secondaryController.rightBumper().whileTrue(Commands.run(() -> feeder.setSpeed(1), feeder).finallyDo(() -> feeder.stop()));
                secondaryController.rightTrigger().whileTrue(Commands.run(() -> shooter.setRPM(1000), shooter).finallyDo(() -> shooter.stop()));



                // ── SysId routines (dev/practice bot only) ────────────────────────────
                if (!Drive.comp) {
                // ── Drivetrain SysID ──────────────────────────────────────────────
                mainController.back().and(mainController.y())
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                mainController.back().and(mainController.x())
                                .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                mainController.start().and(mainController.y())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                mainController.start().and(mainController.x())
                                .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // ── Shooter SysID ─────────────────────────────────────────────────
                secondaryController.back().and(secondaryController.y())
                                .whileTrue(shooter.sysIdDynamic(Direction.kForward));
                secondaryController.back().and(secondaryController.x())
                                .whileTrue(shooter.sysIdDynamic(Direction.kReverse));
                secondaryController.start().and(secondaryController.y())
                                .whileTrue(shooter.sysIdQuasistatic(Direction.kForward));
                secondaryController.start().and(secondaryController.x())
                                .whileTrue(shooter.sysIdQuasistatic(Direction.kReverse));
                }
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}