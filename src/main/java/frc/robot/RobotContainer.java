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
import frc.robot.Constants.OI;
import frc.robot.commands.PointAtHub;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.Constants.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {     
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        //initilaize slew rate limiters
        private final SlewRateLimiter xSlewLimiter = new SlewRateLimiter(OI.slewRate);
        private final SlewRateLimiter ySlewLimiter = new SlewRateLimiter(OI.slewRate);
        private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(OI.rotationSlewRate);

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * OI.deadband)
                        .withRotationalDeadband(MaxAngularRate * OI.deadband) // Add a deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final SendableChooser<Command> autoChooser;
        private final Telemetry logger = new Telemetry();

        private final CommandXboxController xboxController = new CommandXboxController(OI.driverControllerPort);
        public final CommandSwerveDrivetrain drivetrain = new CommandSwerveDrivetrain(
                        TunerConstants.DrivetrainConstants,
                        0, // odometry update frequency (0 = use default)
                        VecBuilder.fill(Drive.odometryXYStdDevs, Drive.odometryXYStdDevs, Drive.odometryYawStdDev),
                        VecBuilder.fill(999, 999, 999), //this is the *default* vision std dev. These values are never used because we always dynamically set it in updateVision()
                        TunerConstants.FrontLeft,
                        TunerConstants.FrontRight,
                        TunerConstants.BackLeft,
                        TunerConstants.BackRight);

        public RobotContainer() {
                configureBindings();
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                // Drivetrain default command
                drivetrain.setDefaultCommand(
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(xSlewLimiter.calculate(-xboxController.getLeftY())
                                                                * MaxSpeed)
                                                .withVelocityY(ySlewLimiter.calculate(-xboxController.getLeftX())
                                                                * MaxSpeed)
                                                .withRotationalRate(m_rotLimiter.calculate(-xboxController.getRightX() 
                                                                * MaxAngularRate))));

                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                RobotModeTriggers.autonomous().onFalse(drivetrain.runOnce(drivetrain::clearFieldPath)); //clear the dashboard visualized paths once auto ends

                xboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                xboxController.b().whileTrue(drivetrain.applyRequest(() -> point
                                .withModuleDirection(new Rotation2d(-xboxController.getLeftY(),
                                                -xboxController.getLeftX()))));



                // reset the field-centric heading on left bumper press(LB)
                xboxController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                //shoot fuel while held; don't have a shooter yet, so currently just point towards hub,
                // translation (deadband + slew + scaling) is handled here and passed into the command
                xboxController.rightTrigger().whileTrue(
                        new PointAtHub(
                                drivetrain,
                                () -> {
                                        double rawForward = -xboxController.getLeftY();
                                        return (Math.abs(rawForward) < OI.deadband) ? 0.0 : xSlewLimiter.calculate(rawForward) * MaxSpeed;
                                },
                                () -> {
                                        double rawLeft = -xboxController.getLeftX();
                                        return (Math.abs(rawLeft) < OI.deadband) ? 0.0 : ySlewLimiter.calculate(rawLeft) * MaxSpeed;
                                }
                        )
                );

                //the following bindings only do anything if drive.comp is false(not in a competition settnig. that boolean has to be manually set)
                if(!Drive.comp){
                        // Run SysId routines when holding back/start and X/Y.
                        // Note that each routine should be run exactly once in a single log.
                        xboxController.back().and(xboxController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                        xboxController.back().and(xboxController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                        xboxController.start().and(xboxController.y())
                                        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                        xboxController.start().and(xboxController.x())
                                        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                        
                        //For testing purposes only: reset position to in front of the center of the red alliance hub, facing red alliance wall(By Apriltags 9 and 10)
                        xboxController.leftTrigger().onTrue(
                                        new InstantCommand(() -> drivetrain.resetPose(
                                                        new Pose2d((492.88 + 15) * 0.0254, (158.32) * 0.0254, //0.0254 converts from in to m
                                                                        Rotation2d.fromDegrees(180)))));      
                        //run wheel characterization
                        xboxController.leftTrigger().and(xboxController.rightTrigger())
                                        .onTrue(new WheelRadiusCharacterization(drivetrain));
                }

                drivetrain.registerTelemetry(logger::telemeterize);
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }       
}
