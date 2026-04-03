package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.kShooter;

/**
 * Velocity-compensated aiming solver.
 *
 * Solves for a "virtual target": the point the robot must aim at so that
 * robot motion carries the game piece into the hub.
 *
 * Iterative solve (converges in 2–4 iterations):
 *   1. Straight-line vector to hub → distance d₀
 *   2. Compute flight time t = flightTime(d₀)
 *   3. Lead offset = robot velocity × t
 *   4. Virtual target = hub − lead offset
 *      (subtracting because we need the piece to arrive at the hub,
 *       not at the virtual target itself)
 *   5. Recompute distance to virtual target → new flight time → repeat
 *   6. Stop when virtual target moves < kAimConvergenceThreshold between
 *      iterations, or after kMaxAimIterations.
 *
 * Flight time formula --------------------------------------------------
 * Flight time is derived from first principles rather than a lookup table,
 * keeping it self-consistent with rpmTable at every distance.
 *
 * We solve for the time the ball reaches goal height (not when it returns
 * to the ground). Vertical kinematics:
 *
 *   y(t) = v_muzzle × sin(θ) × t  −  ½g × t²  =  kGoalHeightDelta
 *
 * Rearranging into standard quadratic form (½g)t² − (v_y)t + h = 0:
 *
 *   t = (v_y − sqrt(v_y² − 2g × h)) / g       ← ascending root (smaller t)
 *
 * where:
 *   v_y = v_muzzle × sin(kLaunchAngleRads)
 *   h   = kGoalHeightDelta  (goal opening minus shooter exit, in metres)
 *
 * If the discriminant goes negative (ball physically cannot reach that
 * height at this RPM/angle), we fall back to the horizontal-range estimate.
 *
 * kLaunchFraction is a single tunable constant (typically 0.75–0.90 for foam
 * game pieces). Measure it once: shoot from a known distance, back-calculate
 * v_muzzle from where the ball lands, then divide by v_surface.
 * 
 * WE Aren't using the math for now. planning on LUT for rpm and ToF, if they're parabolic, we can look at an equation
 */
public final class AimingSolver {

    private AimingSolver() {} // utility class — no instances

    // ── Solve constants ────────────────────────────────────────────────────

    /** Maximum iterations for the velocity-compensation solve. */
    private static final int    kMaxAimIterations        = 6;

    /** Gravitational acceleration (m/s²). */
    private static final double kG = 9.80665;

    /**
     * Vertical height the ball must rise from the shooter exit to the goal
     * opening, in metres.
     *
     *   Goal opening height:  60.00 in  (bottom of ball at goal)
     *   Shooter exit height:   7.85 in  (bottom of ball at launch)
     *   Delta:                52.15 in  = 1.3251 m
     */
    private static final double kGoalHeightDelta = (60.0 - 7.85) * 0.0254; // 1.3251 m

    /**
     * Convergence threshold (meters). The solve stops early when the virtual
     * target moves less than this between consecutive iterations.
     */
    private static final double kAimConvergenceThreshold = 0.01;

    // ── Result record ──────────────────────────────────────────────────────

    /**
     * Output of {@link #solve}.
     *
     * @param aimAngle      field-relative heading (radians) toward the virtual target
     * @param shootDistance distance to the virtual target (meters); used for RPM lookup
     */
    public static final class Solution {
        public final double aimAngle;
        public final double shootDistance;

        Solution(double aimAngle, double shootDistance) {
            this.aimAngle      = aimAngle;
            this.shootDistance = shootDistance;
        }
    }

    // ── Flight time ────────────────────────────────────────────────────────

    /**
     * Computes the time for the ball to reach goal height, derived from
     * vertical kinematics rather than horizontal range.
     *
     * <pre>
     *   shooterRpm = rpmTable(distance) × kGearRatio
     *   v_surface  = shooterRpm / 60 × 2π × kWheelRadius
     *   v_muzzle   = v_surface × kLaunchFraction
     *   v_y        = v_muzzle × sin(kLaunchAngleRads)
     *
     *   Solve  ½g·t² − v_y·t + h = 0  for the ascending root:
     *   t = (v_y − sqrt(v_y² − 2g·h)) / g
     * </pre>
     *
     * Falls back to the horizontal-range estimate if the discriminant is
     * negative (ball cannot physically reach that height at this RPM/angle).
     *
     * @param distance horizontal distance to target (metres)
     * @return flight time in seconds
     */
    public static double flightTime(double distance) {
        // use the table
            return kShooter.flightTimeTable.get(distance);
        
        // return flightTimePhysics(distance);
    }
    /**
     * Physics-derived flight time,fall back
     */
    // static double flightTimePhysics(double distance) {
    //     double motorRpm   = kShooter.rpmTable.get(distance);
    //     double shooterRpm = motorRpm * kShooter.kGearRatio;
    //     double vSurface   = (shooterRpm / 60.0) * (2.0 * Math.PI * kShooter.kWheelRadius);
    //     double vMuzzle    = vSurface * kShooter.kLaunchFraction;

    //     double vY           = vMuzzle * Math.sin(kShooter.kLaunchAngleRads);
    //     double discriminant = vY * vY - 2.0 * kG * kGoalHeightDelta;

    //     if (discriminant >= 0.0) {
    //         return (vY - Math.sqrt(discriminant)) / kG;
    //     }

    //     double vHorizontal = vMuzzle * Math.cos(kShooter.kLaunchAngleRads);
    //     if (vHorizontal < 0.1) return 1.0;
    //     return distance / vHorizontal;
    // }

    // ── Aiming solve ───────────────────────────────────────────────────────

    /**
     * Iteratively solves for the virtual target that accounts for robot motion
     * during projectile flight.
     *
     * <p>The virtual target is the point the robot must aim at so that the game
     * piece — fired toward the virtual target — arrives at the hub after the
     * robot has moved by {@code velocity × flightTime}.
     *
     * <p>Formula per iteration:
     * <pre>
     *   d_n                 = distance(robotPos, virtualTarget_n)
     *   t_n                 = flightTime(d_n)
     *   virtualTarget_{n+1} = hubPos − velocity × t_n
     * </pre>
     *
     * Convergence is declared when the virtual target moves less than
     * {@link #kAimConvergenceThreshold} metres between iterations, or after
     * {@link #kMaxAimIterations} iterations.
     *
     * @param robotPos current robot position (field frame, metres)
     * @param hubPos   hub/goal position (field frame, metres)
     * @param vx       robot velocity X (field frame, m/s)
     * @param vy       robot velocity Y (field frame, m/s)
     * @return solved {@link Solution}
     */
    public static Solution solve(
            Translation2d robotPos,
            Translation2d hubPos,
            double vx,
            double vy) {

        // Seed: aim straight at the hub
        Translation2d virtualTarget = hubPos;

        for (int i = 0; i < kMaxAimIterations; i++) {
            double dx   = virtualTarget.getX() - robotPos.getX();
            double dy   = virtualTarget.getY() - robotPos.getY();
            double dist = Math.hypot(dx, dy);

            double t = flightTime(dist);

            Translation2d nextTarget = new Translation2d(
                hubPos.getX() - vx * t,
                hubPos.getY() - vy * t
            );

            double convergenceDelta = nextTarget.getDistance(virtualTarget);
            virtualTarget = nextTarget;

            if (convergenceDelta < kAimConvergenceThreshold) break;
        }

        double dx    = virtualTarget.getX() - robotPos.getX();
        double dy    = virtualTarget.getY() - robotPos.getY();
        double dist  = Math.hypot(dx, dy);
        double angle = Math.atan2(dy, dx);

        return new Solution(angle, dist);
    }
}