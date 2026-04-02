package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.Constants.kShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

/**
 * Funnel — shoots game pieces from the neutral zone back into the robot's own alliance zone.
 *
 * ── Field geometry ────────────────────────────────────────────────────────────
 *
 *   Blue zone  │  Neutral zone  │  Red zone
 *   x < 5.3    │  5.3 < x < 11.3│  x > 11.3
 *
 * There are two hub structures, one on each alliance's boundary. They sit
 * roughly centered in the Y range of the field (~Y=3.3 to 4.9m). When a robot
 * in the neutral zone wants to shoot back toward its own alliance zone, the hub
 * on its alliance boundary is directly in the way if aimed at mid-field Y.
 *
 * The ball must go AROUND one side of the hub — either below (Y < hubLeftY)
 * or above (Y > hubRightY) — to reach the alliance zone without bouncing off
 * the hub and staying in the neutral zone.
 *
 * ── Aim slot selection ────────────────────────────────────────────────────────
 *
 * Two candidate aim directions are computed each time the command initializes:
 *
 *   "Low" slot:  angle to the low hub corner  (hubLeftY  = 3.3m), minus an
 *                angular clearance buffer (kAngularClearanceRads). This points
 *                the shot to pass below the hub.
 *
 *   "High" slot: angle to the high hub corner (hubRightY = 4.9m), plus an
 *                angular clearance buffer. This points the shot to pass above.
 *
 * The angular clearance is applied in heading-space (radians), not as a flat
 * Y offset at the boundary. This means the physical gap between the aimed
 * trajectory and the hub corner scales correctly with robot distance — a robot
 * far from the hub gets more lateral clearance for the same angular buffer than
 * a robot close to it.
 *
 * The slot requiring the smaller heading change from the robot's current pose
 * is chosen. This is locked for the lifetime of the command to avoid the robot
 * oscillating between slots mid-shot.
 *
 * ── Feed gating ───────────────────────────────────────────────────────────────
 *
 * Feeding is suppressed unless ALL of the following are true (when auto=false):
 *   1. Robot is within the neutral zone (between blueXBoundary and redXBoundary).
 *   2. Shooter is at target RPM.
 *   3. Heading error is within tolerance.
 *
 * When auto=true, boundary and angle checks are skipped — feeding is gated only
 * on shooter RPM. Heading control is also disabled; the robot drives freely.
 *
 * Runs until the driver releases the button.
 */
public class Funnel extends Command {

    // ── Constants ─────────────────────────────────────────────────────────────

    /**
     * Angular clearance added away from each hub corner when computing aim slots.
     *
     * Rather than offsetting the Y aim point by a fixed number of meters, we
     * offset the aim *angle* by this many radians away from the angle that would
     * graze the hub corner. This keeps the physical clearance proportional to
     * robot distance: a robot 5m away gets more lateral margin than one 2m away
     * for the same angular buffer, which is the correct behavior (closer = more
     * dangerous to clip the hub, so aim farther in angle space).
     */
    private static final double kAngularClearanceRads = Math.toRadians(7.5);

    // ── Dependencies ──────────────────────────────────────────────────────────

    private final Shooter m_shooter;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Feeder m_feeder;
    private final DoubleSupplier m_vxSupplier;
    private final DoubleSupplier m_vySupplier;

    /**
     * When true, boundary and angle checks are bypassed. The robot does not
     * attempt to control its heading, and feeding is gated only on shooter RPM.
     * Intended for use in autonomous or situations where the driver has already
     * pre-positioned and aimed the robot.
     */
    private final boolean m_auto;

    private boolean m_useLow;
    private Translation2d m_hubCornerLow;
    private Translation2d m_hubCornerHigh;

    // ── Runtime state ─────────────────────────────────────────────────────────

    /** Detected or inferred alliance, latched at initialize(). */
    private Alliance m_alliance;

    /**
     * The aim angle (radians, field-relative) chosen at initialize() and held
     * fixed for the life of the command. Pointing toward either the low slot
     * (below the hub) or the high slot (above the hub), with angular clearance.
     * Unused when m_auto=true.
     */
    private double m_targetAngleRads;

    /**
     * Heading PID drives the robot to face m_targetAngleRads.
     * Continuous input enabled so it wraps correctly across ±π.
     * //TUNE: gains copied from Shoot as a starting point.
     */
    private final PIDController m_headingPID =
        new PIDController(5.0, 0.0, 0.1);

    private final SwerveRequest.FieldCentric m_fieldCentric =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(
                com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    // ── Constructor ───────────────────────────────────────────────────────────

    /**
     * @param shooter    shooter subsystem — controls flywheel RPM
     * @param drivetrain swerve drivetrain — controls heading and translation
     * @param feeder     feeder subsystem — feeds game pieces into the shooter
     * @param vxSupplier field-relative X velocity from the driver joystick (m/s)
     * @param vySupplier field-relative Y velocity from the driver joystick (m/s)
     * @param auto       if true, skips boundary and angle checks; feeds whenever
     *                   shooter is at RPM regardless of robot position or heading
     */
    public Funnel(
        Shooter shooter,
        CommandSwerveDrivetrain drivetrain,
        Feeder feeder,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier,
        boolean auto
    ) {
        m_shooter    = shooter;
        m_drivetrain = drivetrain;
        m_feeder     = feeder;
        m_vxSupplier = vxSupplier;
        m_vySupplier = vySupplier;
        m_auto       = auto;

        addRequirements(shooter, drivetrain, feeder);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_headingPID.reset();
        m_headingPID.enableContinuousInput(-Math.PI, Math.PI);

        Translation2d robotPos       = m_drivetrain.getState().Pose.getTranslation();
        double        currentHeading = m_drivetrain.getState().Pose.getRotation().getRadians();

        // Determine alliance — use DriverStation if available, else infer from
        // which goal the robot is closer to (same fallback logic as Shoot).
        m_alliance = DriverStation.getAlliance().orElseGet(() ->
            closestAlliance(robotPos));

        // The hub we must avoid sits on OUR alliance's boundary (the X line
        // separating neutral zone from our alliance zone). Its Y extent is
        // [hubLeftY, hubRightY] = [3.3, 4.9].
        double boundaryX = (m_alliance == Alliance.Red)
            ? kShooter.redXBoundary
            : kShooter.blueXBoundary;

        m_hubCornerLow  = new Translation2d(boundaryX, kShooter.hubLeftY);
        m_hubCornerHigh = new Translation2d(boundaryX, kShooter.hubRightY);

        // In auto mode, skip slot selection — heading is not controlled.
        if (!m_auto) {
            double angleToCornerLow  = angleToPoint(robotPos, m_hubCornerLow);
            double angleToCornerHigh = angleToPoint(robotPos, m_hubCornerHigh);

            double angleLow  = angleToCornerLow  - kAngularClearanceRads;
            double angleHigh = angleToCornerHigh + kAngularClearanceRads;

            double errorLow  = Math.abs(Rotation2d.fromRadians(currentHeading)
                                   .minus(Rotation2d.fromRadians(angleLow)).getRadians());
            double errorHigh = Math.abs(Rotation2d.fromRadians(currentHeading)
                                   .minus(Rotation2d.fromRadians(angleHigh)).getRadians());

            m_useLow = errorLow <= errorHigh;
            m_targetAngleRads = m_useLow ? angleLow : angleHigh;

            SmartDashboard.putString("Funnel/ChosenSlot", m_useLow ? "Low (left of hub)" : "High (right of hub)");
            SmartDashboard.putNumber("Funnel/TargetAngleDeg", Math.toDegrees(m_targetAngleRads));
        } else {
            // Still pick a hub corner for distance-based RPM calculation, using
            // whichever corner is closer to the robot's current heading.
            double angleToCornerLow  = angleToPoint(robotPos, m_hubCornerLow);
            double angleToCornerHigh = angleToPoint(robotPos, m_hubCornerHigh);
            double errorLow  = Math.abs(Rotation2d.fromRadians(currentHeading)
                                   .minus(Rotation2d.fromRadians(angleToCornerLow)).getRadians());
            double errorHigh = Math.abs(Rotation2d.fromRadians(currentHeading)
                                   .minus(Rotation2d.fromRadians(angleToCornerHigh)).getRadians());
            m_useLow = errorLow <= errorHigh;

            SmartDashboard.putString("Funnel/ChosenSlot", "Auto (no angle control)");
        }
    }

    @Override
    public void execute() {
        Translation2d robotPos       = m_drivetrain.getState().Pose.getTranslation();
        double        currentHeading = m_drivetrain.getState().Pose.getRotation().getRadians();

        // Set shooter RPM based on distance to the relevant hub corner.
        m_shooter.setShootingDistance(
            m_auto ? 5.5 : robotPos.getDistance(m_useLow ? m_hubCornerLow : m_hubCornerHigh));

        boolean atAngle;
        double pidOutput = 0.0;

        if (m_auto) {
            // No heading control — pass driver rotation through directly (zero here;
            // caller may combine with a separate rotation supplier if needed).
            atAngle = true;
        } else {
            double angleError = Math.abs(
                Rotation2d.fromRadians(currentHeading)
                    .minus(Rotation2d.fromRadians(m_targetAngleRads))
                    .getRadians());

            atAngle   = angleError < kShooter.angleTolerance_Rads;
            pidOutput = m_headingPID.calculate(currentHeading,
                            Rotation2d.fromRadians(m_targetAngleRads).getRadians());

            SmartDashboard.putNumber("Shooter/AngleError", Math.toDegrees(angleError));
        }

        m_drivetrain.setControl(
            m_fieldCentric
                .withVelocityX(m_vxSupplier.getAsDouble())
                .withVelocityY(m_vySupplier.getAsDouble())
                .withRotationalRate(pidOutput)
        );

        boolean clearToFeed = m_auto || isClearToFeed(robotPos);

        if (clearToFeed && m_shooter.atTargetRPM() && atAngle) {
            m_feeder.feed();
        } else {
            m_feeder.stop();
        }

        String status;
        if (!clearToFeed) {
            status = "Not in Neutral Zone";
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
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_feeder.stop();
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /**
     * Returns the field-relative heading angle (radians) from {@code from}
     * pointing toward {@code to}. Uses standard atan2 convention:
     * 0 = +X, π/2 = +Y, wraps in (-π, π].
     */
    private static double angleToPoint(Translation2d from, Translation2d to) {
        Translation2d delta = to.minus(from);
        return Math.atan2(delta.getY(), delta.getX());
    }

    /**
     * Returns true if the robot is in the neutral zone.
     * Not evaluated when m_auto=true.
     */
    private boolean isClearToFeed(Translation2d robotPos) {
        double x = robotPos.getX();
        return x >= kShooter.blueXBoundary && x <= kShooter.redXBoundary;
    }

    /**
     * Fallback alliance detection when DriverStation doesn't report one.
     * Picks whichever alliance goal the robot is physically closer to.
     */
    private static Alliance closestAlliance(Translation2d robotPos) {
        return (robotPos.getDistance(kShooter.kRedHub) <= robotPos.getDistance(kShooter.kBlueHub))
            ? Alliance.Red
            : Alliance.Blue;
    }
}