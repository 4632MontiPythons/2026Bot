package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.MatchInfo;

import static edu.wpi.first.units.Units.*;

/**
 * Command that points the robot toward the hub while still allowing the
 * driver to translate the robot using the left stick. The driver's rotation
 * input is ignored and replaced with a P-controlled rotational rate that
 * turns the robot to face the hub (based on alliance-specific hub coordinates
 * in {@link Constants.Shooter} and our currently estimated pose from {@link CommandSwerveDrivetrain#getEstimatedPose()}).
 */
public class PointAtHub extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier vxSupplier;
    private final DoubleSupplier vySupplier;
    private final CommandXboxController driverController;
    private double hubX;
    private double hubY;

    // Maximum angular speed (match value used in RobotContainer)
    private final double maxAngularRateRadPerSec = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
    

    // Proportional gain for heading controller (rad/s per rad of error)
    private static final double kP_angle = 3.0;

    public PointAtHub(CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, DoubleSupplier vxSupplier, DoubleSupplier vySupplier) {
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;

        addRequirements(drivetrain);
    }


    @Override
    public void initialize() {
        // Use MatchInfo singleton for alliance information (non-blocking)
        MatchInfo.getInstance().ensureInitialized();
        var optAlliance = MatchInfo.getInstance().getOwnAlliance();

        if (optAlliance.isPresent()) {
            Alliance alliance = optAlliance.get();
            if (alliance == Alliance.Red) {
                this.hubX = Constants.Shooter.redGoalX;
                this.hubY = Constants.Shooter.redGoalY;
            } else {
                this.hubX = Constants.Shooter.blueGoalX;
                this.hubY = Constants.Shooter.blueGoalY;
            }
            // Check boundaries and rumble if in neutral zone or wrong alliance zone
            Pose2d pose = drivetrain.getEstimatedPose();
            double x = pose.getX();
            double blueBoundary = Constants.Shooter.blueXBoundary;
            double redBoundary = Constants.Shooter.redXBoundary;

            boolean inNeutral = (x > blueBoundary && x < redBoundary);
            boolean inWrongZone = (alliance == Alliance.Blue && x >= redBoundary) || (alliance == Alliance.Red && x <= blueBoundary);

            if (inNeutral || inWrongZone) {
                rumbleDriver(0.5);
            }
        } else {
            // If we can't determine alliance, choose the closest hub based on current estimated pose
            Pose2d pose = drivetrain.getEstimatedPose();
            double bx = Constants.Shooter.blueGoalX;
            double by = Constants.Shooter.blueGoalY;
            double rx = Constants.Shooter.redGoalX;
            double ry = Constants.Shooter.redGoalY;

            double distToBlue = Math.hypot(pose.getX() - bx, pose.getY() - by);
            double distToRed = Math.hypot(pose.getX() - rx, pose.getY() - ry);

            if (distToRed < distToBlue) {
                this.hubX = rx;
                this.hubY = ry;
            } else {
                this.hubX = bx;
                this.hubY = by;
            }
        }
    }

    private void rumbleDriver(double seconds) {
        // in initialize() where you want to rumble
        var hid = driverController.getHID();
        Commands.sequence(
            Commands.runOnce(() -> { hid.setRumble(RumbleType.kLeftRumble, 1.0); hid.setRumble(RumbleType.kRightRumble, 1.0); }),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> { hid.setRumble(RumbleType.kLeftRumble, 0.0); hid.setRumble(RumbleType.kRightRumble, 0.0); })
        ).schedule();
    }

    @Override
    public void execute() {
        // Current robot pose
        Pose2d pose = drivetrain.getEstimatedPose();

        // Vector from robot to hub 
        double dx = hubX - pose.getX();
        double dy = hubY - pose.getY();

        // Desired heading (angle to hub)
        double desiredYaw = Math.atan2(dy, dx);
        double currentYaw = pose.getRotation().getRadians();

        double angleError = desiredYaw - currentYaw;
        angleError = MathUtil.angleModulus(angleError);

        // P-controller for rotational rate
        double rotRate = kP_angle * angleError;
        // Saturate
        if (rotRate > maxAngularRateRadPerSec) rotRate = maxAngularRateRadPerSec;
        if (rotRate < -maxAngularRateRadPerSec) rotRate = -maxAngularRateRadPerSec;

        // Translation supplied by RobotContainer (already processed: deadband, slew, scaling)
        double vx = vxSupplier.getAsDouble();
        double vy = vySupplier.getAsDouble();

        // Build and apply field-centric request
        var req = new SwerveRequest.FieldCentric()
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withRotationalRate(rotRate)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        drivetrain.setControl(req);
    }

    @Override
    public boolean isFinished() {
        return false; // default command-style: never finishes on its own
    }

    @Override
    public void end(boolean interrupted) {
        // Stop commanding speeds when command ends
        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds());
    }
}



