package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.Drive;
import frc.robot.Constants.OI;
import frc.robot.generated.TunerConstants;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.MetersPerSecond;

/**
 * Swallow driving mode, Velocity-Oriented Drive: the intake always faces the direction the robot is moving.
 * (get it, its a pun, swallow the fuel, and also the unladened swallow. plus fuel slighlt resembles coconuts)
 * By locking the robot's heading to its velocity vector, the full intake width
 * is always perpendicular to travel regardless of which direction the driver steers.
 *
 * FieldCentricFacingAngle is used so the heading controller closes the loop
 * on rotation automatically; the driver only controls X/Y.
 * 
 * Inspired by https://www.chiefdelphi.com/t/frc-95-the-grasshoppers-2026-build-thread/507859/19
 */
public class SwallowIntake extends Command {

    private static final double DEADBAND_SQUARED = OI.deadband * OI.deadband;

    private final CommandSwerveDrivetrain drivetrain;
    private final Intake intake;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;

    private final double maxSpeed;

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(OI.slewRate);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(OI.slewRate);

    private final SwerveRequest.FieldCentricFacingAngle driveRequest =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * OI.deadband)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /**
     * @param drivetrain  Swerve drivetrain subsystem
     * @param intake      Intake subsystem
     * @param xSupplier   Raw joystick forward axis  (negative = forward)
     * @param ySupplier   Raw joystick strafe axis   (negative = left)
     */
    public SwallowIntake(
            CommandSwerveDrivetrain drivetrain,
            Intake intake,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier) {
        this.drivetrain = drivetrain;
        this.intake     = intake;
        this.xSupplier  = xSupplier;
        this.ySupplier  = ySupplier;
        this.maxSpeed   = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

        //PID the field angle
        driveRequest.HeadingController.setPID(
                Drive.swallowHeadingKp,
                Drive.swallowHeadingKi,
                Drive.swallowHeadingKd);
        driveRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drivetrain, intake);
    }

    @Override
    public void initialize() {
        intake.deploy();//ensure its deployed
        //reset slew limiters
        xLimiter.reset(xSupplier.getAsDouble());
        yLimiter.reset(ySupplier.getAsDouble());
    }

    @Override
    public void execute() {
        double rawX = xSupplier.getAsDouble();
        double rawY = ySupplier.getAsDouble();

        // Only update target heading when the stick is outside the deadband
        // otherwise the robot would be unpredicatble when the driver centers the stick.
        Rotation2d targetHeading;
        if (rawX * rawX + rawY * rawY > DEADBAND_SQUARED) {
            // atan2(y, x) gives the field-relative angle of the velocity vector.
            // rawX is forward (field +X) and rawY is left (field +Y).
            targetHeading = new Rotation2d(-rawX, -rawY); //intakes on the back of the bot
        } else {
            //hold whatever heading the controller last locked to.
            targetHeading = drivetrain.getState().Pose.getRotation();
        }

        double vx = xLimiter.calculate(rawX) * maxSpeed;
        double vy = yLimiter.calculate(rawY) * maxSpeed;

        drivetrain.setControl(
                driveRequest
                        .withVelocityX(vx)
                        .withVelocityY(vy)
                        .withTargetDirection(targetHeading));
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}