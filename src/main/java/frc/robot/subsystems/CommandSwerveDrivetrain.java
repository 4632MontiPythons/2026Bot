package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.LimelightHelpers;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private final Field2d m_field = new Field2d(); // dashboard pose visualization
    private RobotConfig ppRobotConfig;



    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    //SysId routine for characterizing translation. This is used to find PID gains for the drive motors.
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    Volts.per(Second).of(Drive.translationRampRate),
                    Volts.of(Drive.translationStep),
                    Seconds.of(Drive.timeout), // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    //SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, //Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;


    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     * drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     * unspecified or set to 0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     * calculation
     * in the form [x, y, theta]ᵀ, with units in
     * meters
     * and radians
     * @param visionStandardDeviation   The standard deviation for vision
     * calculation
     * in the form [x, y, theta]ᵀ, with units in
     * meters
     * and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStdDevs,
            Matrix<N3, N1> visionStdDevs,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStdDevs, visionStdDevs, modules);
        SmartDashboard.putData("Field", m_field);
        com.pathplanner.lib.util.PathPlannerLogging.setLogActivePathCallback(
            (poses) -> m_field.getObject("path").setPoses(poses)
        );
        com.pathplanner.lib.util.PathPlannerLogging.setLogTargetPoseCallback(
            (pose) -> m_field.getObject("targetPose").setPose(pose)
        );

        try {
            ppRobotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("PathPlanner RobotConfig failed to load", e.getStackTrace());
        }

        AutoBuilder.configure( //something to note is that this is just supplying pathplanner with the required functions, not running them
            this::getEstimatedPose,        //pose supplier
            this::resetPose,               //reset pose method
            () -> this.getState().Speeds,  //robot-relative chassis speeds
            (speeds, ff) -> {
                setControl(
                    new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)
                        .withDriveRequestType(DriveRequestType.Velocity)
                );
            },
            Drive.ppController,
            ppRobotConfig,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, //as noted above, this will run when auto starts, so we don't have to be worried about being connected to FMS when booting bot
            this
        );
    }

    /**
     * Returns the current estimated pose of the robot from the CTRE SwerveDrivetrain.
     * @return The Pose2d of the robot.
     */
    public Pose2d getEstimatedPose() {
        return this.getState().Pose; //Return the pose calculated by the underlying TunerSwerveDrivetrain/SwerveDrivetrain
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        System.out.println("Running SysID");
        return m_sysIdRoutineToApply.quasistatic(direction)
            .beforeStarting(SignalLogger::start) // Start logging before test
            .finallyDo((interrupted) -> SignalLogger.stop()); // Stop logging after test
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction)
        .beforeStarting(SignalLogger::start) // Start logging before test
        .finallyDo((interrupted) -> SignalLogger.stop()); // Stop logging after test
    }

    @Override
    public void periodic() {
        updateVision();
        m_field.setRobotPose(getEstimatedPose()); // Update the dashboard field with the pose from the CTRE estimator
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // module encoder values for debugging
        //SmartDashboard.putNumber("FL Steer Angle", getModule(0).getCurrentState().angle.getRotations());
        //SmartDashboard.putNumber("FR Steer Angle", getModule(1).getCurrentState().angle.getRotations());
        //SmartDashboard.putNumber("BL Steer Angle", getModule(2).getCurrentState().angle.getRotations());
        //SmartDashboard.putNumber("BR Steer Angle", getModule(3).getCurrentState().angle.getRotations());
        //SmartDashboard.putNumber("Yaw (deg)", getPigeon2().getYaw().getValue().in(Degrees));
        //SmartDashboard.putNumber("Estimated X (m)", getEstimatedPose().getX());
        //SmartDashboard.putNumber("Estimated Y (m)", getEstimatedPose().getY());
        //SmartDashboard.putNumber("Estimated Rotation (deg)", getEstimatedPose().getRotation().getDegrees());
    }
    public void clearFieldPath() {
        m_field.getObject("path").setPoses();
        m_field.getObject("targetPose").setPoses();
    }

    private void updateVision() {
        //First we are sending our robot's orientation(from pigeon) to the limelight so that it can effectively calculate position
        var pigeon = getPigeon2();
        double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        LimelightHelpers.SetRobotOrientation("limelight",
         pigeon.getYaw().getValueAsDouble(), 
         yawRate, 
         pigeon.getPitch().getValueAsDouble(), 
         0,
         pigeon.getRoll().getValueAsDouble(), 
         0);

        //first we'll check MegaTag1 results for just yaw estimation; we are very very picky about this data because pigeon is generally very accurate for yaw
        //this is mainly just for this year, because of the bump; consider removing this code for future seasons
        var mt1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight"); //grab LL estimate
        boolean useMT1Yaw = false;
        boolean useMT2Pose = false;
        useMT1Yaw = (mt1Result != null
                && mt1Result.tagCount >= Vision.megaTag1MinTagsForYaw
                && mt1Result.avgTagDist < Vision.megaTag1MaxDistance
                && Math.abs(yawRate) < Vision.megaTag1maxYawRate_DegPerSec);

         
        //check megatag2 results
        var mt2Result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight"); //grab LL estimate
        double xyStdDev = 67; //never actually used with this value, but compiler wants us to initiliaze it
        if (mt2Result != null
                && Math.abs(yawRate) < Vision.maxYawRate_DegPerSec
                && mt2Result.avgTagDist < Vision.maxTagDistance_Meters) {

            useMT2Pose=true;
            xyStdDev = (Vision.baseXYStdDev/mt2Result.tagCount)
                            * (1+(Math.abs(yawRate) * Vision.yawRateCoefficent))
                            + (mt2Result.avgTagDist*Vision.stdDevPerMeter);
        }

        //final pose: megatag2 combined with megatag1(if valid)
        if(useMT1Yaw && useMT2Pose){
            System.out.println("Just for testing, REMOVE LATER; Valid MT1 Yaw: " + mt1Result.pose.getRotation().getDegrees()+"Yaw before:" + getEstimatedPose().getRotation());
            Pose2d finalPose = new Pose2d(mt2Result.pose.getTranslation(), mt1Result.pose.getRotation());
            if(!Drive.comp) m_field.getObject("VisionEstimate").setPose(finalPose); //visualize our last valid LL pose estimate on dashboard
            addVisionMeasurement(finalPose, mt2Result.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, Vision.megaTag1YawStdDev));
        }
        else if(useMT2Pose){
            if(!Drive.comp) m_field.getObject("VisionEstimate").setPose(mt2Result.pose); //visualize our last valid LL pose estimate on dashboard
            addVisionMeasurement(mt2Result.pose, mt2Result.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, 9999999));
        } else if (useMT1Yaw && !useMT2Pose) { 
            //extreme edge case, only way i see mt1 being valid but not mt2 is if we are sending wrong yaw to mt2 so it returns null; but that is the exact scenario we need mt1 for
            System.out.println("Just for testing, REMOVE LATER; Valid MT1 Yaw: " + mt1Result.pose.getRotation().getDegrees()+"Yaw before:" + getEstimatedPose().getRotation());
            Pose2d finalPose = new Pose2d(0, 0, mt1Result.pose.getRotation());
            if(!Drive.comp) m_field.getObject("VisionEstimate").setPose(finalPose); //visualize our last valid LL pose estimate on dashboard
            addVisionMeasurement(finalPose, mt1Result.timestampSeconds, VecBuilder.fill(9999999, 9999999, Vision.megaTag1YawStdDev));
        }
    }


    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     * vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     * seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     * measurement
     * in the form [x, y, theta]ᵀ, with units in
     * meters and radians.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }
}