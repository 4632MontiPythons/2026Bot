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
import edu.wpi.first.math.util.Units;

public final class Constants {
    //Prevent instantiation
    private Constants() {
    }

    public static final class Drive {
        public static final double maxAngularRateRadPerSec = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        //Most Drive constants are located in /generated/TunerConstants.java
        public static final double odometryXYStdDevs = 0.03;
        public static final double odometryYawStdDev = Units.degreesToRadians(0.75);
        public static final boolean comp = false; //CHANGE THIS AT COMP
        public static final boolean log = true; //change to true once we get a usb stick for the rio

        //Auto config
        public final static PPHolonomicDriveController ppController =
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), //translation
                new PIDConstants(4.0, 0.0, 0.0)  //rotation
            );

        //sysID
        public static final double translationRampRate = 0.75; //for quasistatic (volts per second)
        public static final double translationStep = 4; //for dynamic (volts)
        public static final double timeout = 7;
        //  assuming max speed of 4.5 m/s, feet traveled during sysID rotuine = 0.615 *timeout^2 *ramprate

    }

    public static final class kVision {
    /* Common constants for photonvision and LL */
        public static final boolean USE_PHOTONVISION = false; //until we actually get the hardware set up
        public static final boolean kDisableVisionVizualization = false;
        
        public static final double kYawRateCoefficent = (1.0/200.0);
        public static final double kTagDistCoefficent = 0.3;

        public static final double kMaxTagDistance_Meters = 4.0;
        public static final double kMaxYawRate_DegPerSec = 200; 

        //for mt1 and photonvision yaw
        public static final int kMinTagsForYaw = 2;
        public static final double kYawMaxTagDistance = 2.0;
        public static final double kYawMaxYawRate_DegPerSec = 50.0;
        public static final double kYawStdDev = Units.degreesToRadians(2.75);


    /* Limelight constants */
        public static final String LL_camName = "limelight";
        public static final double camX = 0.0;     //forward is +X
        public static final double camY = 0.0;     //right is +Y
        public static final double camZ = 0.0;     //up is +Z
        public static final double camRoll = 0.0;  //right wing down is +roll
        public static final double camPitch = 0.0; //nose up is +pitch
        public static final double camYaw = 0.0;   //nose right is +yaw

        public static final double LL_baseXYStdDev = 0.05; //mt2 is quite a bit better than pv so lower std dev

    /*  PhotonVision constants  */
        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        public static final String kPhotonCam1Name = "Camera_Left";
        public static final Transform3d kRobotToPhotonCam1 = new Transform3d(
            new Translation3d(0.0, 0.2, 0.5), 
            new Rotation3d(0, Units.degreesToRadians(-30), 0)
        );

        public static final String kPhotonCam2Name = "Camera_Right";
        public static final Transform3d kRobotToPhotonCam2 = new Transform3d(
            new Translation3d(0.0, -0.2, 0.5), 
            new Rotation3d(0, Units.degreesToRadians(-30), 0)
        );

        public static final double PV_baseXYStdDev = 0.10;
    }

    public static final class OI {
        public static final double deadband = 0.10; //percentage of max speed/rotational rate. e.g. 10% deadband should be 0.10
        public static final int driverControllerPort = 0;
        public static final double slewRate = 6.0; //limits change to (100*k)% per second, meaning would take 1/k seconds to go from requesting 0 to requesting full throttle
        public static final double rotationSlewRate = 10.0;
    }
    public static final class Shooting {
    // Hub coordinates in meters (X, Y)
        public static final double redGoalX = Units.inchesToMeters(469.11);
        public static final double redGoalY = Units.inchesToMeters(158.84);

        public static final double blueGoalX = Units.inchesToMeters(182.11);
        public static final double blueGoalY = Units.inchesToMeters(158.84);

        public static final double redXBoundary = Units.inchesToMeters(470);
        public static final double blueXBoundary = Units.inchesToMeters(180);
        public static final double kPAngle = 3.0; // Proportional gain for heading control when aiming on the move

    }
}