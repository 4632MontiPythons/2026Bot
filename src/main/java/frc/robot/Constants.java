package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.util.Units;
// import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    //Prevent instantiation
    private Constants() {
    }

    public static final class Drive {
        //Most Drive constants are located in /generated/TunerConstants.java
        public static final double odometryXYStdDevs = 0.03;
        public static final double odometryYawStdDev = Units.degreesToRadians(0.75);
        public static final double wheelXtocenter = Units.inchesToMeters(11.125);
        public static final double wheelYtocenter = Units.inchesToMeters(11.125);

        public static final boolean comp = false; //CHANGE THIS AT COMP
        public static final boolean log = false; //change to true once we get a usb stick for the rio

        //Auto config
        public final static PPHolonomicDriveController ppController =
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0), //translation
                new PIDConstants(4.0, 0.0, 0.0)  //rotation
            );

        //sysID
        public static final double translationRampRate = 0.75; //for quasistatic (volts per second)
        public static final double translationStep = 4; //for dynamic (volts)
        public static final double timeout = 10;
        //  assuming max speed of 4.5 m/s, feet traveled during sysID rotuine = 0.615 *timeout^2 *ramprate

    }

    public static final class Vision {
        public static final String camName = "limelight";

        //offset of camera from robot center; used for changing pose estimate of where camera is to robot center
        //proper measurement is very important, especially angle
        public static final double camX = 0.0;     //forward is +X
        public static final double camY = 0.0;     //right is +Y
        public static final double camZ = 0.0;     //up is +Z
        public static final double camRoll = 0.0;  //right wing down is +roll
        public static final double camPitch = 0.0; //nose up is +pitch
        public static final double camYaw = 0.0;   //nose right is +yaw


        //std dev measurements in meters
        public static final double baseXYStdDev = 0.06;
        //divide by number of tags visible
        //multiply by 1+(|yawDegPerSec|*yawRateCoefficent)
        //add avg tag distance * stdDevPerMeter
        public static final double maxTagDistance_Meters = 4.0; //ignore visual measurements when average distance to tags greater than this value.
        public static final double maxYawRate_DegPerSec = 450; //ignore visual measurements when yaw rate is greater than this value
        public static final double yawRateCoefficent = (1.0/200.0);
        public static final double stdDevPerMeter = 0.02; //Std dev increase per meter

        // megatag1 settings, just used for yaw, set it to be very picky. will only really work when close to the tag triplets by the hub
        public static final int megaTag1MinTagsForYaw = 3;
        public static final double megaTag1MaxDistance = 2.0;
        public static final double megaTag1maxYawRate_DegPerSec = 50.0;
        public static final double megaTag1YawStdDev = Units.degreesToRadians(2.0);

    }

    public static final class OI {
        public static final double deadband = 0.10; //percentage of max speed/rotational rate. e.g. 10% deadband should be 0.10
        public static final int driverControllerPort = 0;
        public static final double slewRate = 3.0; //limits change to (100*k)% per second, meaning would take 1/k seconds to go from 0 to full throttle
        public static final double rotationSlewRate = 6.0;
    }
    public static final class Shooter {
        // Hub coordinates in meters (X, Y)
        public static final double redGoalX = Units.inchesToMeters(469.11);
        public static final double redGoalY = Units.inchesToMeters(158.84);

        public static final double blueGoalX = Units.inchesToMeters(182.11);
        public static final double blueGoalY = Units.inchesToMeters(158.84);

    }
}