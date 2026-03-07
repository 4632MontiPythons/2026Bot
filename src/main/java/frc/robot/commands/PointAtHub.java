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

import frc.robot.Constants.Drive;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.MatchInfo;
import frc.robot.Constants.kShooter;
import frc.robot.util.Elastic;
/**
 *  Points robot at hub, taking control of heading, but allowing driver to control translation.
 * Uses feedforward to predict necessary rotation to stay pointed at hub while translating, and PID to correct for any heading errors.
 * Determines which hub to point at based on alliance and estimated pose. If estimated pose is in the wrong zone, it will still point at the correct hub but will rumble the controller and send a notification to alert the driver.
 * If estimated pose is unavailable, it will default to pointing at the closest hub.
 * We don't have a shooter yet, so this is more of a test. Doesn't account for inherited velocity of ball. 
 * Once we have a shooter we will add a Look up table and use ToF times robot velocity to predict where the hub will be when the ball arrives, and point there instead of the actual hub position.
  */
public class PointAtHub extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier vxSupplier;
    private final DoubleSupplier vySupplier;
    private final CommandXboxController driverController;
    private double hubX;
    private double hubY;
    private final Elastic.Notification wrongZoneNotification = new Elastic.Notification(Elastic.NotificationLevel.ERROR, "Wrong Zone", "Estimated Pose is not in shooting zone. Move or manually calibrate pose.");
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);


    public PointAtHub(CommandSwerveDrivetrain drivetrain, CommandXboxController driverController, DoubleSupplier vxSupplier, DoubleSupplier vySupplier) {
        this.drivetrain = drivetrain;
        this.driverController = driverController;
        this.vxSupplier = vxSupplier;
        this.vySupplier = vySupplier;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        if (MatchInfo.getInstance().ensureInitialized()) {
            Alliance alliance = MatchInfo.getInstance().getOwnAlliance().get();
            if (alliance == Alliance.Red) {
                this.hubX = kShooter.redGoalX;
                this.hubY = kShooter.redGoalY;
            } else {
                this.hubX = kShooter.blueGoalX;
                this.hubY = kShooter.blueGoalY;
            }

            Pose2d pose = drivetrain.getState().Pose;
            double x = pose.getX();
            double blueBoundary = kShooter.blueXBoundary;
            double redBoundary = kShooter.redXBoundary;

            boolean inNeutral = (x > blueBoundary && x < redBoundary);
            boolean inWrongZone = (alliance == Alliance.Blue && x >= redBoundary) || (alliance == Alliance.Red && x <= blueBoundary);

            if (inNeutral || inWrongZone) {
                rumbleDriver();
                Elastic.sendNotification(wrongZoneNotification);
            }
        } else {
            // Fallback: choose closest hub
            Pose2d pose = drivetrain.getState().Pose;
            double bx = kShooter.blueGoalX;
            double by = kShooter.blueGoalY;
            double rx = kShooter.redGoalX;
            double ry = kShooter.redGoalY;

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

    private void rumbleDriver() {
        var hid = driverController.getHID();
        Commands.sequence(
            Commands.runOnce(() -> { hid.setRumble(RumbleType.kBothRumble, 1.0); }),
            Commands.waitSeconds(0.5),
            Commands.runOnce(() -> { hid.setRumble(RumbleType.kBothRumble, 0.0); })
        ).schedule();
    }

    @Override
    public void execute() {
        Pose2d pose = drivetrain.getState().Pose;

        //Get Inputs so driver can move 
        double vx = vxSupplier.getAsDouble();
        double vy = vySupplier.getAsDouble();

        //Calculate vector to hub
        double rx = hubX - pose.getX();
        double ry = hubY - pose.getY();
        double distSq = rx * rx + ry * ry;

        // Calculate Angular Feedforward (compensating for lateral translation)
        // if you're translating laterally, you need to apply a constant rotational velocity to stay facing a static object.
        //this will help to predict rather than just react. results in smoother rotation and better tracking compared to pure PID.
        // Formula: FF = (ry * vx - rx * vy) / radius^2
        double omegaFF = 0;
        omegaFF = (ry * vx - rx * vy) / distSq;

        // Calculate PID Feedback(compensate for heading error)
        double desiredYaw = Math.atan2(ry, rx);
        double currentYaw = pose.getRotation().getRadians();
        double angleError = MathUtil.angleModulus(desiredYaw - currentYaw);
        
        double pidOutput = kShooter.kPAngle * angleError;

        //Combine PID and FF, clamp to max angular rate
        double totalRotRate = pidOutput + omegaFF;
        totalRotRate = MathUtil.clamp(totalRotRate, -Drive.maxAngularRateRadPerSec, Drive.maxAngularRateRadPerSec);

        //Apply rotation and translation requests
        drivetrain.setControl(driveRequest
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(totalRotRate));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds());
    }
}