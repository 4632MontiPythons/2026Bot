package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.swerve.SwerveRequest;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.DoubleSupplier;

public class WheelRadiusCharacterization extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    
    // Configurable rotation speed (rad/s). 
    // Slower = more accurate data, but takes longer. 1.0 is a good balance.
    private static final double ROTATION_SPEED = 1.0; 
    
    private final SwerveRequest.RobotCentric spinRequest = new SwerveRequest.RobotCentric()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(ROTATION_SPEED); 

    private double lastYawRadians;
    private double accumulatedYawRadians = 0;
    
    private final double[] lastWheelDistancesMeters = new double[4];
    private double accumulatedWheelDistanceMeters = 0;

    // The radius from the center of the robot to the center of the modules
    private final double driveBaseRadiusMeters;

    public WheelRadiusCharacterization(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);

        // 1. Get the drive base radius directly from TunerConstants.
        // We use FrontLeft, assuming the robot is symmetrical or rectangular.
        // Phoenix 6 constants (LocationX/Y) are always in Meters.
        this.driveBaseRadiusMeters = Math.hypot(
            TunerConstants.FrontLeft.LocationX, 
            TunerConstants.FrontLeft.LocationY
        );
    }

    @Override
    public void initialize() {
        accumulatedYawRadians = 0;
        accumulatedWheelDistanceMeters = 0;

        // Get initial gyro state
        lastYawRadians = drivetrain.getState().Pose.getRotation().getRadians();

        // Get initial wheel distances
        var modulePositions = drivetrain.getState().ModulePositions;
        for (int i = 0; i < 4; i++) {
            lastWheelDistancesMeters[i] = modulePositions[i].distanceMeters;
        }

        System.out.println("=== Wheel Radius Characterization START ===");
        System.out.printf("Drive Base Radius: %.4f meters%n", driveBaseRadiusMeters);
        System.out.println("Starting rotation... please wait for 10 rotations.");
    }

    @Override
    public void execute() {
        drivetrain.setControl(spinRequest);

        // ---- Robot Rotation (Gyro) ----
        double currentYawRadians = drivetrain.getState().Pose.getRotation().getRadians();
        double deltaYaw = currentYawRadians - lastYawRadians;

        // Handle Gyro wrapping (-PI to PI)
        if (deltaYaw > Math.PI) deltaYaw -= 2 * Math.PI;
        if (deltaYaw < -Math.PI) deltaYaw += 2 * Math.PI;

        accumulatedYawRadians += Math.abs(deltaYaw);
        lastYawRadians = currentYawRadians;

        // ---- Wheel Distance (Odometry) ----
        double wheelDeltaSum = 0;
        var modulePositions = drivetrain.getState().ModulePositions;
        for (int i = 0; i < 4; i++) {
            double currentDist = modulePositions[i].distanceMeters;
            wheelDeltaSum += Math.abs(currentDist - lastWheelDistancesMeters[i]);
            lastWheelDistancesMeters[i] = currentDist;
        }
        
        // Average the distance of all 4 modules
        accumulatedWheelDistanceMeters += wheelDeltaSum / 4.0;
    }

    @Override
    public boolean isFinished() {
        // Stop after 10 full rotations (20 * PI) to ensure a large sample size
        return accumulatedYawRadians >= (2 * Math.PI * 10);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds());

        if (interrupted || accumulatedWheelDistanceMeters <= 0.1) {
             System.out.println("=== Characterization INTERRUPTED or FAILED ===");
             return;
        }

        // 1. Calculate the actual distance the wheels *should* have traveled 
        //    based on how much the robot actually rotated.
        //    Arc Length = Angle (radians) * Radius
        double actualDistanceMeters = accumulatedYawRadians * driveBaseRadiusMeters;

        // 2. Calculate the adjustment factor
        //    Factor = Actual / Measured
        double adjustmentFactor = actualDistanceMeters / accumulatedWheelDistanceMeters;
        
        // 3. Apply to current radius
        //    We read the current radius directly from TunerConstants
        double currentRadiusInches = TunerConstants.kWheelRadius.in(Inches); 
        double newWheelRadiusInches = currentRadiusInches * adjustmentFactor;

        System.out.println("\n========== CHARACTERIZATION RESULTS ==========");
        System.out.printf("Rotations:      %.2f%n", accumulatedYawRadians / (2*Math.PI));
        System.out.printf("Wheel Dist:     %.3f m%n", accumulatedWheelDistanceMeters);
        System.out.printf("Gyro Dist:      %.3f m%n", actualDistanceMeters);
        System.out.println("----------------------------------------------");
        System.out.printf("Adjustment:     %.5f%n", adjustmentFactor);
        System.out.printf("Old Radius:     %.5f inches%n", currentRadiusInches);
        System.out.printf("NEW RADIUS:     %.5f inches%n", newWheelRadiusInches);
        System.out.println("----------------------------------------------");
        System.out.println("Copy/Paste this line into TunerConstants.java:");
        System.out.printf("public static final Distance kWheelRadius = Inches.of(%.5f);%n", newWheelRadiusInches);
        System.out.println("==============================================\n");
    }
}