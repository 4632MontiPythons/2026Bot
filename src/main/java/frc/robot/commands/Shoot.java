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
 * Teleop: runs until interrupted (button released), or self-terminates when
 * the hub is imminently going inactive. The feed gate and self-termination both
 * use the flight time table so behavior scales with distance.
 *
 * Auto: self-terminates after a fixed timeout (expectedShootTimeSecs)
 */
public class Shoot extends Command {

    // ── Dependencies ──────────────────────────────────────────────────────────

    private final Shooter m_shooter;
    private final Feeder m_feeder;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final boolean m_isAuto;

    private final DoubleSupplier m_vxSupplier;
    private final DoubleSupplier m_vySupplier;
    private boolean m_hubWasActive = false;
    /** Expected duration (seconds) to drain the full hopper. Auto only. */
    private final double m_expectedShootTimeSecs;

    // ── Runtime state ─────────────────────────────────────────────────────────

    private final Timer m_commandTimer = new Timer();

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

    private boolean m_isBraking = false;

    /**
     * Set in execute(), consumed by isFinished().
     * Avoids recomputing position/distance/flight time in a second code path.
     */
    private boolean m_shouldFinish = false;

    // ── Constructors ──────────────────────────────────────────────────────────

    /** Teleop constructor. */
    public Shoot(
        Shooter shooter,
        Feeder feeder,
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier
    ) {
        this(shooter, feeder, drivetrain, vxSupplier, vySupplier, false, 0.0);
    }

    /** Full constructor — teleop and auto. */
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
        m_headingPID.reset();
        m_isBraking    = false;
        m_shouldFinish = false;

        m_alliance = DriverStation.getAlliance().orElseGet(() ->
            closestAlliance(m_drivetrain.getState().Pose.getTranslation()));
        m_goalPos = (m_alliance == Alliance.Red) ? kShooter.kRedHub : kShooter.kBlueHub;

        Translation2d initPos = m_drivetrain.getState().Pose.getTranslation();
        m_shooter.setShootingDistance(initPos.getDistance(m_goalPos));

        m_hubWasActive = false;
    }

    @Override
    public void execute() {
        // ── Single state snapshot ────────────────────────────────────────────
        // Grab once so all reads come from the same tick's data.
        var state = m_drivetrain.getState();
        double vx = state.Speeds.vxMetersPerSecond;
        double vy = state.Speeds.vyMetersPerSecond;
        Translation2d currentPos = state.Pose.getTranslation();

        // ── Aiming solve ─────────────────────────────────────────────────────
        AimingSolver.Solution aim = AimingSolver.solve(currentPos, m_goalPos, vx, vy);
        double targetAngle   = aim.aimAngle;
        double shootDistance = aim.shootDistance;

    

        // ── Hub status ───────────────────────────────────────────────────
        boolean hubShootWindowOpen = HubSchedule.isHubShootWindowOpen(m_alliance);
        //only cancel command if went from active -> inactive. want ot be able to hold shoot button before window opens and have it warm up and aim
        m_shouldFinish = m_hubWasActive && !hubShootWindowOpen;

        
        m_hubWasActive = hubShootWindowOpen;


        // ── Drive control ────────────────────────────────────────────────────
        double angleError = Math.abs(state.Pose.getRotation()
                .minus(Rotation2d.fromRadians(targetAngle)).getRadians());
        boolean atAngle = angleError < kShooter.angleTolerance_Rads;
        boolean joystickStationary =
                Math.hypot(m_vxSupplier.getAsDouble(), m_vySupplier.getAsDouble()) < 0.05; //TUNE

        if (joystickStationary && atAngle) {
            m_isBraking = true;
        } else if (!joystickStationary || angleError > kShooter.angleTolerance_Rads * 2) {
            m_isBraking = false;
        }

        if (m_isBraking) {
            m_drivetrain.setControl(m_brake);
        } else {
            double currentHeading = state.Pose.getRotation().getRadians();
            double pidOutput      = m_headingPID.calculate(currentHeading, targetAngle);

            m_drivetrain.setControl(
                m_fieldCentric
                    .withVelocityX(m_vxSupplier.getAsDouble())
                    .withVelocityY(m_vySupplier.getAsDouble())
                    .withRotationalRate(pidOutput)
            );
        }

        m_shooter.setShootingDistance(shootDistance);

        // ── Feed gate ────────────────────────────────────────────────────────
        boolean clearToFeed = hubShootWindowOpen;



        if (clearToFeed && m_shooter.atTargetRPM() && atAngle) {
            m_feeder.feed();
        } else {
            m_feeder.stop();
        }

        // ── Finish flag (consumed by isFinished) ─────────────────────────────
        if (m_isAuto) {
            m_shouldFinish = m_commandTimer.get() >= m_expectedShootTimeSecs;
        } else {
            m_shouldFinish = !hubShootWindowOpen;
        }



        // ── Dashboard ────────────────────────────────────────────────────────
        String status;
        if (!clearToFeed) {
            if (!hubShootWindowOpen) {
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

        SmartDashboard.putString("Shooter/Status",      status);
        SmartDashboard.putNumber("Shooter/AngleError",  Math.toDegrees(angleError));
        SmartDashboard.putNumber("Shooter/ElapsedTime", m_commandTimer.get());
    }

    @Override
    public boolean isFinished() {
        return m_shouldFinish;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private static Alliance closestAlliance(Translation2d robotPos) {
        return (robotPos.getDistance(kShooter.kRedHub) <= robotPos.getDistance(kShooter.kBlueHub))
            ? Alliance.Red
            : Alliance.Blue;
    }

    // /**
    //  * @param hubInactive pre-computed
    //  */
    // private boolean isClearToFeed(Translation2d robotPos) {
    //     double x = robotPos.getX();
    //     return (m_alliance == Alliance.Red)
    //         ? x >= kShooter.redXBoundary
    //         : x <= kShooter.blueXBoundary;
    // }
}