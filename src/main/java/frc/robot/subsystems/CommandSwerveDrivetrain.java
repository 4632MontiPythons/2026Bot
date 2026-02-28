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
import frc.robot.Constants.Drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private final Field2d m_field = new Field2d(); // dashboard pose visualization
    // private final Vision m_vision; // Vision subsystem instance
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
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        //Apply control request
                        setControl(m_translationCharacterization.withVolts(output));
                    },
                    null,
                    this));

    //SysId routine for characterizing steer. This is used to find PID gains for the steer motors.
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, //Use default ramp rate (1 V/s)
                    Volts.of(Drive.translationStep), // Use dynamic voltage of 7 V
                    Seconds.of(Drive.timeout),
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
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
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
        
        // Initialize Vision
        // m_vision = new Vision(this, m_field);

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
            () -> this.getState().Pose,        //pose supplier
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
            () -> DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red, //as noted above, this will run when auto starts, so we don't have to be worried about being connected to FMS when booting bot
            this
        );
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
        return m_sysIdRoutineToApply.quasistatic(direction)
            .beforeStarting(() -> {
                System.out.println("Running SysID Quasistatic");
                SignalLogger.start();
            })
            .finallyDo((interrupted) -> SignalLogger.stop());
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction)
            .beforeStarting(() -> {
                System.out.println("Running SysID Dynamic");
                SignalLogger.start();
            })
            .finallyDo((interrupted) -> SignalLogger.stop());
    }

    @Override
    public void periodic() {
        // m_vision.update();

        m_field.setRobotPose(this.getState().Pose); // Update the dashboard field with the pose from the CTRE estimator
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
    
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs, String cameraName) {
    
    super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);

    // Logging the measurement along with the camera name
    try {
        var logger = frc.robot.Telemetry.getInstance();
        if (logger != null) {
            logger.logVisionMeasurement(
                visionRobotPoseMeters, 
                Utils.fpgaToCurrentTime(timestampSeconds), 
                visionMeasurementStdDevs,
                cameraName
            );
        }
    } catch (Exception e) {
        System.err.println("CommandSwerveDrivetrain: failed to log vision measurement: " + e.getMessage());
    }
}
}