package frc.robot.subsystems;

import static frc.robot.Constants.kVision.*;


import edu.wpi.first.math.VecBuilder;
import frc.robot.util.LimelightHelpers;
/* 
 * Limelight on shooter side
 */
public class Vision {
    private final CommandSwerveDrivetrain m_drivetrain;
    
    
    public Vision(CommandSwerveDrivetrain drivetrain) {
        this.m_drivetrain = drivetrain;

        LimelightHelpers.setCameraPose_RobotSpace(
            LL_camName, camX, camY, camZ,
            camRoll, camPitch, camYaw);
    
    }

    public void update() {
        var pigeon = m_drivetrain.getPigeon2();
        double yawRate = pigeon.getAngularVelocityZWorld().getValueAsDouble();
        if (Math.abs(yawRate) > kMaxYawRate_DegPerSec) return;
        updateLimelight(
            m_drivetrain.getState().Pose.getRotation().getDegrees(), // pose estimate yaw
            yawRate,
            pigeon.getPitch().getValueAsDouble(),
            pigeon.getRoll().getValueAsDouble()
        );
    }

    private void updateLimelight(double yaw, double yawRate, double pitch, double roll) {
        LimelightHelpers.SetRobotOrientation(LL_camName, yaw, yawRate, pitch, 0, roll, 0);
        
        var mt2Result = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LL_camName);
        var mt1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_camName);
        
        if (mt2Result != null && mt2Result.tagCount > 0 && mt2Result.avgTagDist < kMaxTagDistance_Meters) {
            double mult = (mt2Result.avgTagDist * kTagDistCoefficent + Math.abs(yawRate) * kYawRateCoefficent);
            double xyStdDev = (LL_mt2baseStdDev / mt2Result.tagCount) * (1 + mult);

            if (mt2Result.pose.getX() > 0 && mt2Result.pose.getX() < 16.5 && 
                mt2Result.pose.getY() > 0 && mt2Result.pose.getY() < 8.2) {
                m_drivetrain.addVisionMeasurement(
                            mt2Result.pose, 
                            mt2Result.timestampSeconds, 
                            VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY)
                    );
                }
        }
            

        if (mt1Result != null && mt1Result.tagCount >= kMinTagsForYaw && mt1Result.avgTagDist < kYawMaxTagDistance && Math.abs(yawRate) < kYawMaxYawRate_DegPerSec) {
            m_drivetrain.addVisionMeasurement(
                mt1Result.pose,
                mt1Result.timestampSeconds,
                VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, kYawStdDev)
            );
        }
    }
}