package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public final class Constants {
    private Constants() {
    }

    public static final class Drive {
        public static final double maxAngularRateRadPerSec = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        public static final double odometryXYStdDevs = 0.03;
        public static final double odometryYawStdDev = Units.degreesToRadians(0.75);
        public static final boolean comp = false; //CHANGE THIS AT COMP
        public static final boolean log = false; //off this year cause we cant fit a usb stick under the chassis

        public final static PPHolonomicDriveController ppController =
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(4.0, 0.0, 0.0)
            );

        public static final double translationRampRate = 0.75;
        public static final double translationStep = 4;
        public static final double timeout = 7;
    }

    public static final class kVision {
        public static final boolean USE_PHOTONVISION = false;
        public static final boolean kDisableVisionVizualization = false;

        public static final double kYawRateCoefficent = (1.0 / 200.0);
        public static final double kTagDistCoefficent = 0.3;

        public static final double kMaxTagDistance_Meters = 4.0;
        public static final double kMaxYawRate_DegPerSec = 200;
        public static final double kMaxSingleTagPoseAmbiguity = 0.15;
        public static final int kMinTagsForYaw = 2;
        public static final double kYawMaxTagDistance = 2.0;
        public static final double kYawMaxYawRate_DegPerSec = 50.0;
        public static final double kYawStdDev = Units.degreesToRadians(3);

        public static final String LL_camName = "limelight";
        public static final double camX = 0.0;
        public static final double camY = 0.0;
        public static final double camZ = 0.0;
        public static final double camRoll = 0.0;
        public static final double camPitch = 0.0;
        public static final double camYaw = 0.0;

        public static final double LL_mt2baseStdDev = 0.05;

        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static final String kPhotonCam1Name = "PV_Intake";
        public static final Transform3d kRobotToPhotonCam1 = new Transform3d(
            new Translation3d(0.0, 0.2, 0.5),
            new Rotation3d(0, Units.degreesToRadians(-30), 0)
        );

        public static final String kPhotonCam2Name = "PV_Left";
        public static final Transform3d kRobotToPhotonCam2 = new Transform3d(
            new Translation3d(0.0, -0.2, 0.5),
            new Rotation3d(0, Units.degreesToRadians(-30), 0)
        );

        public static final double PV_baseXYStdDev = 0.10;
    }

    public static final class OI {
        public static final double deadband = 0.10;
        public static final int driverControllerPort = 0;
        public static final double slewRate = 10;
    }

    public static final class kShooter {
        public static final int shooterMotorID = -1;

        public static final Translation2d kRedGoal = new Translation2d(11.915394, 4.034536);
        public static final Translation2d kBlueGoal = new Translation2d(4.625594, 4.034536);

        public static final double redXBoundary = 11.3;
        public static final double blueXBoundary = 5.3;
        public static final double rpmTolerance = 100;
        public static final double angleTolerance_Rads = Units.degreesToRadians(1);
        public static final double angleTolerance_RadsPerSec = Units.degreesToRadians(5);

        public static final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
        static {
            // Distance (m) → Motor shaft RPM (NOT flywheel RPM). Placeholders.
            rpmTable.put(1.0, 2500.0);
            rpmTable.put(2.0, 3200.0);
            rpmTable.put(2.5, 3600.0);
            rpmTable.put(3.0, 3900.0);
            rpmTable.put(3.5, 4200.0);
            rpmTable.put(4.0, 4600.0);
            rpmTable.put(5.0, 5500.0);
        }

        public static final double kEmptyCurrentThreshold = 3.0;
        public static final double kEmptySettleTime = 0.5;
        public static final double kEarlyWindowSecs = 3.0;
        public static final double kLateWindowSecs = 2.0;
    }

    public static final class kIntake {
        public static final int intakeMotorID = -1;
        public static final int solenoidL_Forward = -1;
        public static final int solenoidL_Reverse = -1;
        public static final int solenoidR_Forward = -1;
        public static final int solenoidR_Reverse = -1;
        public static final double intakeSpeed = 0.5;
        public static final int motorCurrentLimit = 30;
        public static final int PCM_CAN_ID = -1;
    }

    public static final class kFeeder {
        public static final int feederMotorID = -1;
        public static final boolean inverted = false;
        public static final double feedSpeed = 0.5;
        public static final double reverseSpeed = -0.4; // negative = reverse direction
        public static final double jamCurrentThreshold = 35.0;
        public static final double reverseTimeSec = 0.3;
        public static final double retryTimeSec = 0.5;
        public static final double jamClearSettleTime = 0.1;
        public static final int maxJamRetries = 3;
    }
}