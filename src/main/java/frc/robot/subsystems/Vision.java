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

public class Vision {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Field2d m_field;
    
    private final PhotonCamera m_cameraLeft;
    private final PhotonCamera m_cameraRight;

    private final PhotonPoseEstimator m_estimatorLeft;
    private final PhotonPoseEstimator m_estimatorRight;
    
    public Vision(CommandSwerveDrivetrain drivetrain, Field2d field) {
        this.m_drivetrain = drivetrain;
        this.m_field = field;

        LimelightHelpers.setCameraPose_RobotSpace(
            LL_camName, camX, camY, camZ,
            camRoll, camPitch, camYaw);
        
        m_cameraLeft = new PhotonCamera(kPhotonCam1Name);
        m_cameraRight = new PhotonCamera(kPhotonCam2Name);

        m_estimatorLeft = new PhotonPoseEstimator(kTagLayout, kRobotToPhotonCam1);
        m_estimatorRight = new PhotonPoseEstimator(kTagLayout, kRobotToPhotonCam2);
    }

    public void update() {
        var pigeon = m_drivetrain.getPigeon2();
        double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        if(yawRate>kMaxYawRate_DegPerSec) return;
        updateLimelight(
            pigeon.getYaw().getValueAsDouble(), 
            yawRate, 
            pigeon.getPitch().getValueAsDouble(), 
            pigeon.getRoll().getValueAsDouble()
        );

        if (USE_PHOTONVISION) {
            processPhotonCamera(m_cameraLeft, m_estimatorLeft, yawRate, "PhotonLeft");
            processPhotonCamera(m_cameraRight, m_estimatorRight, yawRate, "PhotonRight");
        }
    }

    private void processPhotonCamera(PhotonCamera camera, PhotonPoseEstimator estimator, double yawRate, String vizName) {
        var results = camera.getAllUnreadResults(); //this grabs all unread results, and clears the stored queue.
        if (results.isEmpty()) return;

        // Since we're running 30-60 fps we should be good with just grabbing most recent.
        PhotonPipelineResult result = results.get(results.size() - 1);
        if (!result.hasTargets()) return;

        Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);
        if (visionEst.isEmpty()) {
            visionEst = estimator.estimateLowestAmbiguityPose(result);
        }

        if (visionEst.isPresent()) {
            EstimatedRobotPose estimatedPose = visionEst.get();
            Pose2d pose2d = estimatedPose.estimatedPose.toPose2d();
            
            //get avg tag dist:
            double avgDist = 0;
            var targets = estimatedPose.targetsUsed;
            for (var target : targets) {
                avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
            }
            avgDist /= targets.size();

            //calc xy std dev
            double mult = (avgDist* kTagDistCoefficent + Math.abs(yawRate)* kYawRateCoefficent);
            double xyStdDev = (PV_baseXYStdDev / targets.size())*(1+mult);
            
            //Trust yaw under strict conditions
            double thetaStdDev = targets.size() >= kMinTagsForYaw && avgDist< kYawMaxTagDistance ? kYawStdDev : Double.POSITIVE_INFINITY;

            if (!Drive.comp && !kDisableVisionVizualization) m_field.getObject(vizName).setPose(pose2d);

            m_drivetrain.addVisionMeasurement(
                pose2d, 
                estimatedPose.timestampSeconds, 
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev),
                vizName
            );
        }
    }

    private void updateLimelight(double yaw, double yawRate, double pitch, double roll) {
        //Send orientation to LL for MegaTag2
        LimelightHelpers.SetRobotOrientation(LL_camName, yaw, yawRate, pitch, 0, roll, 0);
        
        //Grab both estimates
        var mt2Result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_camName);
        var mt1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_camName);
        
        //Use MegaTag2 for Translation
        if (mt2Result != null && mt2Result.tagCount > 0 && mt2Result.avgTagDist < kMaxTagDistance_Meters) {
            
            double mult = (mt2Result.avgTagDist * kTagDistCoefficent + Math.abs(yawRate) * kYawRateCoefficent);
            double xyStdDev = (LL_baseXYStdDev / mt2Result.tagCount) * (1 + mult);

            m_drivetrain.addVisionMeasurement(
                mt2Result.pose, 
                mt2Result.timestampSeconds, 
                VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY),
                LL_camName + "_MT2"
            );

            if (!Drive.comp && !kDisableVisionVizualization) m_field.getObject("LL_MT2_Pose").setPose(mt2Result.pose);
        }

        //MegaTag1 for Yaw, strict requriements
        if (mt1Result != null && mt1Result.tagCount >= kMinTagsForYaw && mt1Result.avgTagDist < kYawMaxTagDistance) {

            m_drivetrain.addVisionMeasurement(
                mt1Result.pose,
                mt1Result.timestampSeconds,
                VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, kYawStdDev),
                LL_camName + "_MT1"
            );
        }
    }
}