package frc.robot.subsystems;

import static frc.robot.Constants.kVision.*;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Constants.Drive;
import frc.robot.util.LimelightHelpers;
/* One camera on intake side, will be used for fuel detection during auto, otherwise apriltags
 * Limelight on shooter side, apriltags
 * One camera on right side, used for apriltags.
 */
public class Vision {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Field2d m_field;
    
    private final PhotonCamera m_cameraIntake;
    private final PhotonCamera m_cameraRight;

    private final PhotonPoseEstimator m_estimatorIntake;
    private final PhotonPoseEstimator m_estimatorRight;
    
    public Vision(CommandSwerveDrivetrain drivetrain, Field2d field) {
        this.m_drivetrain = drivetrain;
        this.m_field = field;

        LimelightHelpers.setCameraPose_RobotSpace(
            LL_camName, camX, camY, camZ,
            camRoll, camPitch, camYaw);
        
        m_cameraIntake = new PhotonCamera(kPhotonCam1Name);
        m_cameraRight = new PhotonCamera(kPhotonCam2Name);

        m_estimatorIntake = new PhotonPoseEstimator(kTagLayout, kRobotToPhotonCam1);
        m_estimatorRight = new PhotonPoseEstimator(kTagLayout, kRobotToPhotonCam2);
    }

    public void update() {
        var pigeon = m_drivetrain.getPigeon2();
        double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        if (Math.abs(yawRate) > kMaxYawRate_DegPerSec) return;
        updateLimelight(
            pigeon.getYaw().getValueAsDouble(), 
            yawRate, 
            pigeon.getPitch().getValueAsDouble(), 
            pigeon.getRoll().getValueAsDouble()
        );

        if (USE_PHOTONVISION) {
            processPhotonResults(m_cameraIntake, m_estimatorIntake, yawRate, kPhotonCam1Name);
            processPhotonResults(m_cameraRight, m_estimatorRight, yawRate, kPhotonCam2Name);
        }
    }

    private void processPhotonResults(PhotonCamera camera, PhotonPoseEstimator estimator, double yawRate, String vizName) {
        var results = camera.getAllUnreadResults();
        if (results.isEmpty()) return;

        // Change: Process all unread frames instead of only the most recent one.
        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) continue;

            Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
            
            if (visionEst.isEmpty()) {
                if (!Drive.comp && result.targets.size() >= 2) System.out.println("PV: Multi-tag pose estimation failed with multiple tags");
                if (result.getTargets().get(0).getPoseAmbiguity() < kMaxSingleTagPoseAmbiguity) visionEst = estimator.estimateLowestAmbiguityPose(result);
            }

            if (visionEst.isPresent()) {
                EstimatedRobotPose estimatedPose = visionEst.get();
                Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
                
                double avgDist = 0;
                var targets = estimatedPose.targetsUsed;
                for (var target : targets) {
                    avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
                }
                avgDist /= targets.size();
                
                double mult = (avgDist * kTagDistCoefficent + Math.abs(yawRate) * kYawRateCoefficent);
                double xyStdDev = (PV_baseXYStdDev / targets.size()) * (1 + mult);
                
                double thetaStdDev = targets.size() >= kMinTagsForYaw && avgDist < kYawMaxTagDistance ? kYawStdDev : Double.POSITIVE_INFINITY;

                if (!Drive.comp && !kDisableVisionVizualization) m_field.getObject(vizName).setPose(pose2d);

                m_drivetrain.addVisionMeasurement(
                    pose2d, 
                    estimatedPose.timestampSeconds, 
                    VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
                );
            }
        }
    }

    private void updateLimelight(double yaw, double yawRate, double pitch, double roll) {
        LimelightHelpers.SetRobotOrientation(LL_camName, yaw, yawRate, pitch, 0, roll, 0);
        
        var mt2Result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_camName);
        var mt1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_camName);
        
        if (mt2Result != null && mt2Result.tagCount > 0 && mt2Result.avgTagDist < kMaxTagDistance_Meters) {
            double mult = (mt2Result.avgTagDist * kTagDistCoefficent + Math.abs(yawRate) * kYawRateCoefficent);
            double xyStdDev = (LL_mt2baseStdDev / mt2Result.tagCount) * (1 + mult);

            m_drivetrain.addVisionMeasurement(
                mt2Result.pose, 
                mt2Result.timestampSeconds, 
                VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY)
            );

            if (!Drive.comp && !kDisableVisionVizualization) m_field.getObject("LL_MT2_Pose").setPose(mt2Result.pose);
        }

        if (mt1Result != null && mt1Result.tagCount >= kMinTagsForYaw && mt1Result.avgTagDist < kYawMaxTagDistance) {
            m_drivetrain.addVisionMeasurement(
                mt1Result.pose,
                mt1Result.timestampSeconds,
                VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, kYawStdDev)
            );
        }
    }
}