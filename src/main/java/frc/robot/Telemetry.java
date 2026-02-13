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
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Telemetry {
    private static Telemetry s_instance;
    private DoubleArrayLogEntry m_visionPoseLog;
    private DoubleArrayLogEntry m_visionStdDevLog;
    private DoubleLogEntry m_visionTimestampLog;
    private StringLogEntry m_visionCameraNameLog; // Added for DataLog
    private DoubleLogEntry m_batteryLog;

    public static Telemetry getInstance() {
        if (s_instance == null) s_instance = new Telemetry();
        return s_instance;
    }

    public Telemetry() {
        if (Constants.Drive.log) {
            var log = DataLogManager.getLog(); 
            m_visionPoseLog = new DoubleArrayLogEntry(log, "Vision/Pose");
            m_visionStdDevLog = new DoubleArrayLogEntry(log, "Vision/StdDevs");
            m_visionTimestampLog = new DoubleLogEntry(log, "Vision/Timestamp");
            m_visionCameraNameLog = new StringLogEntry(log, "Vision/CameraName"); // Initialize Log
            
            DataLogManager.logNetworkTables(true); 
            m_batteryLog = new DoubleLogEntry(log, "System/BatteryVoltage");
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
    
    /** * Telemeterize the swerve drive state. 
     */
    public void telemeterize(SwerveDriveState state) {
        if (m_batteryLog != null) {
            m_batteryLog.append(edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());
        }
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveModuleStates.set(state.ModuleStates);
        driveModuleTargets.set(state.ModuleTargets);
        driveModulePositions.set(state.ModulePositions);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);
    }

    /**
     * Log vision measurements with source camera identification.
     */
    public void logVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> visionStdDevs, String cameraName) {
        long timestampMicro = (long) (timestampSeconds * 1e6);

        // DataLog updates
        if (Constants.Drive.log) {
            m_visionPoseLog.append(new double[] {
                visionPose.getX(), 
                visionPose.getY(), 
                visionPose.getRotation().getDegrees()
            }, timestampMicro);

            if (visionStdDevs != null) {
                m_visionStdDevLog.append(new double[] {
                    visionStdDevs.get(0, 0),
                    visionStdDevs.get(1, 0),
                    visionStdDevs.get(2, 0)
                }, timestampMicro);
            }

            m_visionTimestampLog.append(timestampSeconds, timestampMicro);
            m_visionCameraNameLog.append(cameraName, timestampMicro);
        }
    }
}