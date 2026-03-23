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
 * ── Heading control ───────────────────────────────────────────────────────────
 *
 * A PID controller drives the robot's heading toward the chosen fixed aim angle.
 * No orbital feedforward or velocity compensation is applied (unlike Shoot):
 * the aim window on either side of the hub is wide, so shoot-on-the-move
 * precision is not required and adding it would risk drifting the aim back into
 * the hub.
 *
 * ── Feed gating ───────────────────────────────────────────────────────────────
 *
 * Feeding is suppressed unless ALL of the following are true:
 *   1. Robot is within the neutral zone (between blueXBoundary and redXBoundary).
 *   3. Shooter is at target RPM.
 *   4. Heading error is within tolerance.
 *
 * ── Termination ───────────────────────────────────────────────────────────────
 * runs
 * until the driver releases the button.
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
     *
     * TUNE: 5° is a reasonable starting point. Increase if balls are clipping
     * the hub, decrease if the shot arc is hitting the alliance boundary walls.
     */
    private static final double kAngularClearanceRads = Math.toRadians(10.0);

    /**
     * The neutral zone is the strip of field between the two alliance boundaries.
     * A robot must be inside this strip to be allowed to feed (shoot).
     *
     * Blue robots funnel toward x = blueXBoundary (their own side, lower X).
     * Red  robots funnel toward x = redXBoundary  (their own side, higher X).
     *
     * Both boundaries are already defined in kShooter; no new constants needed.
     */

    // ── Dependencies ──────────────────────────────────────────────────────────

    private final Shooter m_shooter;
    private final CommandSwerveDrivetrain m_drivetrain;
    private final Feeder m_feeder;
    private final DoubleSupplier m_vxSupplier;
    private final DoubleSupplier m_vySupplier;
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
     */
    public Funnel(
        Shooter shooter,
        CommandSwerveDrivetrain drivetrain,
        Feeder feeder,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier
    ) {
        m_shooter    = shooter;
        m_drivetrain = drivetrain;
        m_feeder     = feeder;
        m_vxSupplier = vxSupplier;
        m_vySupplier = vySupplier;

        addRequirements(shooter, drivetrain, feeder);
    }

    // ── Command lifecycle ─────────────────────────────────────────────────────

    @Override
    public void initialize() {
        m_headingPID.reset();
        m_headingPID.enableContinuousInput(-Math.PI, Math.PI);

        Translation2d robotPos      = m_drivetrain.getState().Pose.getTranslation();
        double        currentHeading = m_drivetrain.getState().Pose.getRotation().getRadians();

        // Determine alliance — use DriverStation if available, else infer from
        // which goal the robot is closer to (same fallback logic as Shoot).
        m_alliance = DriverStation.getAlliance().orElseGet(() ->
            closestAlliance(robotPos));

        // The hub we must avoid sits on OUR alliance's boundary (the X line
        // separating neutral zone from our alliance zone). Its Y extent is
        // [hubLeftY, hubRightY] = [3.3, 4.9].
        //
        // boundaryX is the X coordinate of that boundary line.
        double boundaryX = (m_alliance == Alliance.Red)
            ? kShooter.redXBoundary
            : kShooter.blueXBoundary;

        // Compute the angle from the robot to each hub corner.
        // hubLeftY  = 3.3 → the "low"  corner (smaller Y, below center)
        // hubRightY = 4.9 → the "high" corner (larger  Y, above center)
        m_hubCornerLow  = new Translation2d(boundaryX, kShooter.hubLeftY);
        m_hubCornerHigh = new Translation2d(boundaryX, kShooter.hubRightY);

        double angleToCornerLow  = angleToPoint(robotPos, m_hubCornerLow);
        double angleToCornerHigh = angleToPoint(robotPos, m_hubCornerHigh);

        // Subtract clearance from the low-corner angle  → aim passes BELOW the hub.
        // Add    clearance to   the high-corner angle   → aim passes ABOVE the hub.
        // The clearance is in radians so the physical margin scales with distance.
        double angleLow  = angleToCornerLow  - kAngularClearanceRads;
        double angleHigh = angleToCornerHigh + kAngularClearanceRads;

        // Pick whichever slot requires the smallest heading change from current pose.
        // Locked for the duration of the command to prevent mid-shot slot switching.
        double errorLow  = Math.abs(Rotation2d.fromRadians(currentHeading)
                               .minus(Rotation2d.fromRadians(angleLow)).getRadians());
        double errorHigh = Math.abs(Rotation2d.fromRadians(currentHeading)
                               .minus(Rotation2d.fromRadians(angleHigh)).getRadians());

        m_useLow = errorLow <= errorHigh;
        m_targetAngleRads = m_useLow ? angleLow : angleHigh;

        SmartDashboard.putString("Funnel/ChosenSlot", m_useLow ? "Low (left of hub)" : "High (right of hub)");
        SmartDashboard.putNumber("Funnel/TargetAngleDeg", Math.toDegrees(m_targetAngleRads));
    }

    @Override
    public void execute() {
        Translation2d robotPos       = m_drivetrain.getState().Pose.getTranslation();
        double        currentHeading = m_drivetrain.getState().Pose.getRotation().getRadians();

        double angleError = Math.abs(
            Rotation2d.fromRadians(currentHeading)
                .minus(Rotation2d.fromRadians(m_targetAngleRads))
                .getRadians());

        boolean atAngle = angleError < kShooter.angleTolerance_Rads;
        double pidOutput = m_headingPID.calculate(currentHeading, Rotation2d.fromRadians(m_targetAngleRads).getRadians(););
        m_drivetrain.setControl(
            m_fieldCentric
                .withVelocityX(m_vxSupplier.getAsDouble())
                .withVelocityY(m_vySupplier.getAsDouble())
                .withRotationalRate(pidOutput)
        );
        // Set shooter RPM //TUNE
        m_shooter.setShootingDistance(
            0.6* robotPos.getDistance(m_useLow ? m_hubCornerLow : m_hubCornerHigh)); //not trying to get up into a hub, just over to other side
        boolean clearToFeed = isClearToFeed(robotPos);

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
        SmartDashboard.putNumber("Shooter/AngleError", Math.toDegrees(angleError));
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
     * Returns true if the robot is in the neutral zone and the hub is active.
     *
     * Neutral zone is the strip between blueXBoundary (5.3) and redXBoundary (11.3).
     * Both alliances must be within this strip to funnel — Blue robots are moving
     * toward lower X, Red robots toward higher X, but both start somewhere in the
     * middle and the same neutral-zone check applies to both.
     */
    private boolean isClearToFeed(Translation2d robotPos) {
        double x = robotPos.getX();
        boolean inNeutralZone = x >= kShooter.blueXBoundary && x <= kShooter.redXBoundary;
        return inNeutralZone;
    }

    /**
     * Fallback alliance detection when DriverStation doesn't report one.
     * Picks whichever alliance goal the robot is physically closer to.
     */
    private static Alliance closestAlliance(Translation2d robotPos) {
        return (robotPos.getDistance(kShooter.kRedGoal) <= robotPos.getDistance(kShooter.kBlueGoal))
            ? Alliance.Red
            : Alliance.Blue;
    }
}