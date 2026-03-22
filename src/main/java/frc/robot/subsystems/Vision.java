    package frc.robot.subsystems;

    import static frc.robot.Constants.kVision.*;


    import edu.wpi.first.math.VecBuilder;
    import frc.robot.util.LimelightHelpers;
    /* 
    * Limelight on shooter side
    */
    public class Vision {
        private final CommandSwerveDrivetrain m_drivetrain;
        
        private double lastTimestamp = -1.0;
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

        if (mt2Result == null || mt2Result.tagCount == 0) return;
        if (mt2Result.timestampSeconds <= lastTimestamp) return;
        lastTimestamp = mt2Result.timestampSeconds;

        // --- MT2: XY position ---
        if (mt2Result.avgTagDist < kMaxTagDistance_Meters &&
            mt2Result.pose.getX() > 0 && mt2Result.pose.getX() < kFieldLengthMeters &&
            mt2Result.pose.getY() > 0 && mt2Result.pose.getY() < kFieldWidthMeters) {

            double mult = mt2Result.avgTagDist * kTagDistCoefficent + Math.abs(yawRate) * kYawRateCoefficent;
            double xyStdDev = LL_mt2baseStdDev * (1 + mult) / mt2Result.tagCount;

            m_drivetrain.addVisionMeasurement(
                mt2Result.pose,
                mt2Result.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, Double.POSITIVE_INFINITY)
            );
        }

        // --- MT1: Yaw only ---
        if (mt2Result.tagCount >= kMinTagsForYaw &&
            mt2Result.avgTagDist < kYawMaxTagDistance &&
            Math.abs(yawRate) < kYawMaxYawRate_DegPerSec) {

            var mt1Result = LimelightHelpers.getBotPoseEstimate_wpiBlue(LL_camName);
            if (mt1Result != null) {
                m_drivetrain.addVisionMeasurement(
                    mt1Result.pose,
                    mt1Result.timestampSeconds,
                    VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, kYawStdDev)
                );
            }
        }
    }
}