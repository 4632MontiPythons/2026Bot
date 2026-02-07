package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Telemetry {
    private static Telemetry s_instance;

    private DoubleArrayLogEntry m_visionPoseLog;
    private DoubleArrayLogEntry m_visionStdDevLog;
    private DoubleLogEntry m_visionTimestampLog;

    public static Telemetry getInstance() {
        if (s_instance == null) s_instance = new Telemetry();
        return s_instance;
    }

    public Telemetry() {
        if (Constants.Drive.log) {
            var log = DataLogManager.getLog(); //starts the log and gets the instance
            m_visionPoseLog = new DoubleArrayLogEntry(log, "Vision/Pose");
            m_visionStdDevLog = new DoubleArrayLogEntry(log, "Vision/StdDevs");
            m_visionTimestampLog = new DoubleLogEntry(log, "Vision/Timestamp");
            //tell the log to record everything in NetworkTables:
            DataLogManager.logNetworkTables(true); 
        }
    }

    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("DriveState");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleStates = driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModuleState> driveModuleTargets = driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();
    private final StructArrayPublisher<SwerveModulePosition> driveModulePositions = driveStateTable.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();

    /* Vision NetworkTables */
    private final NetworkTable visionTable = inst.getTable("Vision");
    private final StructPublisher<Pose2d> visionPosePub = visionTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final DoublePublisher visionTimestampPub = visionTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher visionStdDevXPub = visionTable.getDoubleTopic("StdDevX").publish();
    private final DoublePublisher visionStdDevYPub = visionTable.getDoubleTopic("StdDevY").publish();
    private final DoublePublisher visionStdDevThetaPub = visionTable.getDoubleTopic("StdDevTheta").publish();

    /** * Telemeterize the swerve drive state. 
     * Since NT logging is enabled, simply setting the NT values handles the file logging too.
     */
    public void telemeterize(SwerveDriveState state) {
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);
    }

    // Log vision measurement
    public void logVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> visionStdDevs) {
        // Convert seconds to microseconds for the DataLog timestamp
        long timestampMicro = (long) (timestampSeconds * 1e6);

        // Append Pose: [X, Y, RotationDegrees]
        m_visionPoseLog.append(new double[] {
            visionPose.getX(), 
            visionPose.getY(), 
            visionPose.getRotation().getDegrees()
        }, timestampMicro);

        // Append StdDevs: [X, Y, Theta]
        if (visionStdDevs != null) {
            m_visionStdDevLog.append(new double[] {
                visionStdDevs.get(0, 0),
                visionStdDevs.get(1, 0),
                visionStdDevs.get(2, 0)
            }, timestampMicro);
        }

        m_visionTimestampLog.append(timestampSeconds, timestampMicro);
    }
}