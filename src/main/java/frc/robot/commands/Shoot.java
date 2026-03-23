package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AimingSolver;
import frc.robot.util.HubSchedule;
import java.util.function.DoubleSupplier;

/**
 * Aims at the goal, spins up the shooter, and feeds game pieces.
 *
 * Velocity compensation (shoot on the move) --------------------------------
 * Delegates to {@link AimingSolver} to resolve a velocity-compensated virtual
 * target and shooting distance each tick. See that class for full derivation.
 *
 * Orbital feedforward -------------------------------------------------------
 * The facing-angle request includes an orbital feedforward term so the heading
 * controller pre-leads the angle target when the robot is moving tangentially.
 * Without it, the gyro-based heading PID would always lag behind the required
 * angle as the robot sweeps around the goal.
 *
 * Derivation:
 *   dθ/dt = v_tangential / r
 *   where v_tangential is the component of robot velocity perpendicular to the
 *   robot→virtual-target vector, and r is the distance to the virtual target.
 *   We negate because if the target angle increases CCW, the robot must rotate
 *   CW to keep facing it.
 *
 * Teleop vs Auto -----------------------------------------------------------
 * In teleop (isAuto = false): runs until interrupted (button released), or
 * self-terminates when the hub is imminently going inactive.
 * Feeding is gated: if outside the alliance shooting boundary OR the hub is
 * currently inactive, feeding is suppressed while aiming continues.
 *
 * In auto (isAuto = true): self-terminates when hopper is empty or a hard
 * timeout fires.
 *
 * Empty detection ----------------------------------------------------------
 * Detected via RPM drops. kShotSampleCount consecutive above-threshold samples
 * confirm a shot. Two settle windows:
 *   Inside expected window  (elapsed >= expectedTime - kEarlyWindowSecs): kEmptySettleTime_InWindow
 *   Outside expected window:                                               kEmptySettleTime_OutWindow
 * Hard timeout: (expectedShootTimeSecs + kLateWindowSecs)
 */
public class Shoot extends Command {

    // ── Dependencies ──────────────────────────────────────────────────────────

    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final boolean m_isAuto;
    private boolean m_isBraking = false;

    /**
     * Joystick velocity suppliers (meters per second, field-relative).
     * Always {@code () -> 0.0} in auto.
     */
    private final DoubleSupplier m_vxSupplier;
    private final DoubleSupplier m_vySupplier;

    /**
     * Expected duration (seconds) to drain the full hopper.
     * Only used in auto mode.
     */
    private final double m_expectedShootTimeSecs;

    private static int s_totalFuelCount = 0; // persists across command instances
    private double m_baselineRPM = 0.0;

    // ── Runtime state ─────────────────────────────────────────────────────────

    private final Timer m_commandTimer = new Timer();
    private final Timer m_emptyTimer   = new Timer();
    private boolean m_emptyTimerRunning;
    private double  m_emptySettleTime;

    private boolean m_shotEverDetected = false;
    private int     m_shotSampleCount  = 0;

    private final SwerveRequest.FieldCentric m_fieldCentric =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);


    // TUNE
    private final edu.wpi.first.math.controller.PIDController m_headingPID =
        new edu.wpi.first.math.controller.PIDController(5.0, 0.0, 0.1);

    {
        m_headingPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

    private Alliance m_alliance;
    private Translation2d m_goalPos;

    // ── Constructors ──────────────────────────────────────────────────────────

    /**
     * Teleop constructor: full shoot-on-the-move.
     *
     * @param shooter    shooter subsystem
     * @param feeder     feeder subsystem
     * @param drivetrain swerve drivetrain
     * @param vxSupplier field-relative X velocity from joystick (m/s)
     * @param vySupplier field-relative Y velocity from joystick (m/s)
     */
    public Shoot(
        Shooter shooter,
        Feeder feeder,
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier
    ) {
        this(shooter, feeder, drivetrain, vxSupplier, vySupplier, false, 0.0);
    }

    /**
     * Full constructor — supports both teleop shoot-on-the-move and auto.
     *
     * @param shooter               shooter subsystem
     * @param feeder                feeder subsystem
     * @param drivetrain            swerve drivetrain
     * @param vxSupplier            field-relative X velocity supplier (m/s);
     *                              may be {@code () -> 0.0} for stationary auto
     * @param vySupplier            field-relative Y velocity supplier (m/s);
     *                              may be {@code () -> 0.0} for stationary auto
     * @param isAuto                if true, self-terminates when hopper is empty
     * @param expectedShootTimeSecs estimated seconds to drain the full hopper;
     *                              only used when isAuto is true
     */
    public Shoot(
        Shooter shooter,
        Feeder feeder,
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier,
        boolean isAuto,
        double expectedShootTimeSecs
    ) {
        m_shooter               = shooter;
        m_feeder                = feeder;
        m_drivetrain            = drivetrain;
        m_vxSupplier            = vxSupplier;
        m_vySupplier            = vySupplier;
        m_isAuto                = isAuto;
        m_expectedShootTimeSecs = expectedShootTimeSecs;

        addRequirements(shooter, feeder, drivetrain);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_commandTimer.restart();
        m_emptyTimer.reset();
        m_emptyTimerRunning = false;
        m_emptySettleTime   = kShooter.kEmptySettleTime_OutWindow;
        m_shotEverDetected  = false;
        m_shotSampleCount   = 0;

        m_headingPID.reset();
        m_baselineRPM = 0.0;

        SmartDashboard.putNumber("Shooter/TotalFuelFired", s_totalFuelCount);

        m_alliance = DriverStation.getAlliance().orElseGet(() ->
            closestAlliance(m_drivetrain.getState().Pose.getTranslation()));
        m_goalPos = (m_alliance == Alliance.Red) ? kShooter.kRedGoal : kShooter.kBlueGoal;

        // shooter RPM from starting pose (no velocity yet)
        Translation2d initPos = m_drivetrain.getState().Pose.getTranslation();
        m_shooter.setShootingDistance(initPos.getDistance(m_goalPos));
    }

    @Override
    public void execute() {
        double vx = m_drivetrain.getState().Speeds.vxMetersPerSecond;
        double vy = m_drivetrain.getState().Speeds.vyMetersPerSecond;
        
        // Velocity-compensated aiming solve 
        Translation2d currentPos = m_drivetrain.getState().Pose.getTranslation();
        AimingSolver.Solution aim = AimingSolver.solve(currentPos, m_goalPos, vx, vy);

        double targetAngle   = aim.aimAngle;
        double shootDistance = aim.shootDistance;

        // Orbital feedforward against the virtual-target vector
        // Recomputed each tick from the solved virtual-target direction so it
        // remains correct as the robot moves and the virtual target shifts.
        double orbitalFeedforwardRadPerSec = 0.0;
        if (shootDistance > 0.1) {
            double unitX = Math.cos(targetAngle);
            double unitY = Math.sin(targetAngle);
            double vTangential = vx * unitY - vy * unitX;
            orbitalFeedforwardRadPerSec = -(vTangential / shootDistance);
        }



        double angleError = Math.abs(m_drivetrain.getState().Pose.getRotation()
                    .minus(Rotation2d.fromRadians(targetAngle)).getRadians());
        boolean atAngle = angleError < kShooter.angleTolerance_Rads;
        boolean joystickStationary = Math.hypot(m_vxSupplier.getAsDouble(), m_vySupplier.getAsDouble()) < 1e-6;
        
        // 5. Apply drivetrain request

        if (joystickStationary && angleError < kShooter.angleTolerance_Rads) {
            m_isBraking = true;
        } else if(!joystickStationary || angleError < kShooter.angleTolerance_Rads *2) m_isBraking = false;

        if(m_isBraking) m_drivetrain.setControl(m_brake);
        else{
            double currentHeading = m_drivetrain.getState().Pose.getRotation().getRadians();
            double pidOutput      = m_headingPID.calculate(currentHeading, targetAngle);
            double rotationalRate = pidOutput + orbitalFeedforwardRadPerSec;

            m_drivetrain.setControl(
                m_fieldCentric
                    .withVelocityX(m_vxSupplier.getAsDouble())
                    .withVelocityY(m_vySupplier.getAsDouble())
                    .withRotationalRate(rotationalRate)
            );
        }

        // Update shooter RPM from velocity-compensated distance each tick
        m_shooter.setShootingDistance(shootDistance);

        // ── 6. Feed once aimed, up to speed, and clear to shoot ───────────────
        boolean clearToFeed = m_isAuto || isClearToFeed(currentPos);

        if (clearToFeed && m_shooter.atTargetRPM() && atAngle) {
            m_feeder.feed();
        } else {
            m_feeder.stop();
        }

        // ── 7. Empty-hopper detection (auto only) ─────────────────────────────
        if (m_isAuto) {
            updateEmptyDetection();
        }
        String status;
        if (!clearToFeed) {
            if (HubSchedule.isHubInactive(m_alliance)) {
                status = "Hub Inactive";
            } else {
                status = "Out of Bounds";
            }
        } else if (!m_shooter.atTargetRPM()) {
            status = "RPM too low";
        } else if (!atAngle) {
            status = "Adjusting Angle";
        } else {
            status = "Firing";
        }
        SmartDashboard.putString("Shooter/Status", status);
    }

    @Override
    public boolean isFinished() {
        if (!m_isAuto) {
            return HubSchedule.isHubImminentlyInactive(m_alliance, kShooter.kTimeToScore);
        }

        boolean hopperEmpty = m_shotEverDetected
            && m_emptyTimerRunning
            && m_emptyTimer.hasElapsed(m_emptySettleTime);
        boolean timedOut = m_commandTimer.get()
            >= (m_expectedShootTimeSecs + kShooter.kLateWindowSecs);

        return hopperEmpty || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private static Alliance closestAlliance(Translation2d robotPos) {
        return (robotPos.getDistance(kShooter.kRedGoal) <= robotPos.getDistance(kShooter.kBlueGoal))
            ? Alliance.Red
            : Alliance.Blue;
    }

    private boolean isClearToFeed(Translation2d robotPos) {
        double x = robotPos.getX();
        boolean inBoundary = (m_alliance == Alliance.Red)
            ? x >= kShooter.redXBoundary
            : x <= kShooter.blueXBoundary;
        return inBoundary && !HubSchedule.isHubInactive(m_alliance);
    }

    /**
     * RPM-drop based empty-hopper detection. Called every tick in auto only.
     * Latches a baseline RPM on first feed, detects per-ball drops, and starts
     * the settle timer after the last confirmed shot.
     */
    private void updateEmptyDetection() {
        double elapsed = m_commandTimer.get();
        boolean inDetectionWindow = elapsed >= (m_expectedShootTimeSecs - kShooter.kEarlyWindowSecs);
        m_emptySettleTime = inDetectionWindow
            ? kShooter.kEmptySettleTime_InWindow
            : kShooter.kEmptySettleTime_OutWindow;

        if (m_feeder.isFeeding()) {
            double measuredRPM = m_shooter.getMeasuredRPM();

            // Latch baseline on first feed tick
            if (m_baselineRPM < 1.0) {
                m_baselineRPM = measuredRPM;
            }

            boolean dropDetected = (m_baselineRPM - measuredRPM) > kShooter.kShotRpmDrop;

            if (dropDetected) {
                m_shotSampleCount++;
            } else {
                // Slowly track baseline upward as flywheel recovers between balls
                m_baselineRPM = Math.min(Math.max(m_baselineRPM, measuredRPM), m_shooter.getTargetRPM());
                m_shotSampleCount = 0;
            }
        } else {
            m_shotSampleCount = 0;
        }

        boolean shotConfirmed = m_shotSampleCount >= kShooter.kShotSampleCount;

        if (shotConfirmed) {
            m_shotEverDetected  = true;
            m_shotSampleCount   = 0;
            m_emptyTimerRunning = false;
            m_baselineRPM       = 0.0; // reset so next ball re-latches after recovery
            s_totalFuelCount++;
            SmartDashboard.putNumber("Shooter/TotalFuelFired", s_totalFuelCount);
        } else if (m_shotEverDetected && !m_emptyTimerRunning) {
            m_emptyTimer.restart();
            m_emptyTimerRunning = true;
        }
    }
}