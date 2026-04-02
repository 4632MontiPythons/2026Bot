package frc.robot;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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

        public final static PPHolonomicDriveController ppController =
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(4.0, 0.0, 0.0)
            );

        public static final double translationRampRate = 0.75;
        public static final double translationStep = 4;
        public static final double timeout = 7;

        // Swallow (intake-locked) heading PID
        public static final double swallowHeadingKp = 5.0;  //TUNE
        public static final double swallowHeadingKi = 0.0; 
        public static final double swallowHeadingKd = 0.2;
    }

    public static final class kVision {
        // yaw updated with mt1 - strict requirements. 
        public static final double kYawRateCoefficent = (1.0 / 200.0);
        public static final double kTagDistCoefficent = 0.15;

        public static final double kMaxTagDistance_Meters = 4.0;
        public static final double kMaxYawRate_DegPerSec = 200;
        public static final double kMaxSingleTagPoseAmbiguity = 0.15;
        public static final int kMinTagsForYaw = 2;
        public static final double kYawMaxTagDistance = 2.5;
        public static final double kYawMaxYawRate_DegPerSec = 50.0;
        public static final double kYawStdDev = Units.degreesToRadians(3);

        public static final String LL_camName = "limelight";
        public static final double camX = 0.305; //12 in forward of robot center
        public static final double camY = 0.257; //10 1/8 in to the right
        public static final double camZ = 0.324; //12.75 in up
        public static final double camRoll = 0.0;
        public static final double camPitch = 21.0;
        public static final double camYaw = 0.0;

        public static final double LL_mt2baseStdDev = 0.05;

        public static final double kFieldLengthMeters = 16.54;
        public static final double kFieldWidthMeters = 8.07;
    }

    public static final class OI {
        public static final double deadband = 0.10;
        public static final int driverControllerPort = 0;
        public static final double slewRate = 10;
    }

    public static final class kShooter {
        public static final int shooterMotorID = 15;
        public static final double kMaxMotorRPM = 5500.0;
        public static final double kWarmUpRPM = 2000; //TUNE
        public static final Translation2d kRedHub = new Translation2d(11.915394, 4.034536);
        public static final Translation2d kBlueHub = new Translation2d(4.625594, 4.034536);
        public static final double kGearRatio = 2;
        public static final double kLaunchFraction = 0.7; //TUNE. for ToF 
        public static final double kWheelRadius = 0.0762;
        public static final double kLaunchAngleRads = Math.toRadians(60.0);
        public static final double redXBoundary = 11.3;
        public static final double blueXBoundary = 5.3;
        public static final double hubLeftY= 3.3;
        public static final double hubRightY= 4.9;
        public static final double rpmTolerance = 100; //TUNE?
        public static final double angleTolerance_Rads = Units.degreesToRadians(1.0); //TUNE?
        // public static final Translation2d kShooterOffset = new Translation2d(0.34, 0.0); not used anywhere

        public static final InterpolatingDoubleTreeMap rpmTable = new InterpolatingDoubleTreeMap();
        static {
            // Distance (m) → Motor shaft RPM (NOT flywheel RPM). Placeholders. //TUNE
            rpmTable.put(1.0, 2500.0);
            rpmTable.put(2.0, 3200.0);
            rpmTable.put(2.5, 3600.0);
            rpmTable.put(3.0, 3900.0);
            rpmTable.put(3.5, 4200.0);
            rpmTable.put(4.0, 4600.0);
            rpmTable.put(5.0, 5500.0);
        }
        public static final InterpolatingDoubleTreeMap flightTimeTable = new InterpolatingDoubleTreeMap();
        static {
            // Distance (m) -> flight time (s)
            //TUNE
            flightTimeTable.put(1.5, 0.38);
            flightTimeTable.put(2.0, 0.44);
            flightTimeTable.put(2.5, 0.49);
            flightTimeTable.put(3.0, 0.54);
            flightTimeTable.put(3.5, 0.59);
            flightTimeTable.put(4.0, 0.64);
        }

    }

    public static final class kIntake {
        public static final int intakeMotorID = 10;
        public static final int forwardChannel = 0;
        public static final int reverseChannel = 1;
        public static final double intakeSpeed = 1;
        public static final int motorCurrentLimit = 40;
        public static final int PCM_CAN_ID = 21;
    }

    public static final class kFeeder {
        public static final int feederMotorID = 11;
        public static final boolean inverted = true;
        public static final double feedSpeed = 0.5; //TUNE
    }
}