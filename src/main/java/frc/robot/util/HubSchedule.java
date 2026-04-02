package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Tracks hub activity state throughout a match.
 *
 * - Both hubs are ACTIVE during AUTO (0–20s), TRANSITION (130–140s), and ENDGAME (0–30s).
 * - During TELEOP shifts (130s → 30s), exactly one hub is inactive at a time.
 * - The alliance that wins AUTO (or is randomly selected) has their hub INACTIVE in SHIFT 1,
 *   then hub activity alternates every shift (4 total shifts, 25s each).
 * - FMS sends which alliance is inactive first via Game Specific Message:
 *     'R' = Red inactive in SHIFT 1, 'B' = Blue inactive in SHIFT 1.
 *
 * Timing (matchTime counts down):
 *   SHIFT 1: 130–105
 *   SHIFT 2: 105–80
 *   SHIFT 3:  80–55
 *   SHIFT 4:  55–30
 *
 * HubState:
 *   STRICTLY_ACTIVE — hub is definitively active (green)
 *   MARGIN_ACTIVE   — in early/late shoot margin, not yet strictly active (yellow)
 *   INACTIVE        — hub is inactive, do not shoot (red)
 */
public final class HubSchedule {
    private HubSchedule() {}

    // ── Margins ───────────────────────────────────────────────────────────────

    /** Start shooting this many seconds before the strict active window opens. */
    private static final double SHOOT_EARLY_SECS = 2.0;

    /**
     * Keep shooting this many seconds after the strict active window closes.
     * 1.9 instead of 2.0 to account for FMS rounding
     */
    private static final double SHOOT_LATE_SECS = 1.9;

    // ── Shift windows ─────────────────────────────────────────────────────────

    private static final class ShiftWindow {
        final double high;
        final double low;
        final Alliance inactive;

        ShiftWindow(double high, double low, Alliance inactive) {
            this.high = high;
            this.low = low;
            this.inactive = inactive;
        }
    }

    private static final ShiftWindow[] SCHEDULE_RED_FIRST = {
        new ShiftWindow(130.0, 105.0, Alliance.Red),
        new ShiftWindow(105.0,  80.0, Alliance.Blue),
        new ShiftWindow( 80.0,  55.0, Alliance.Red),
        new ShiftWindow( 55.0,  30.0, Alliance.Blue),
    };

    private static final ShiftWindow[] SCHEDULE_BLUE_FIRST = {
        new ShiftWindow(130.0, 105.0, Alliance.Blue),
        new ShiftWindow(105.0,  80.0, Alliance.Red),
        new ShiftWindow( 80.0,  55.0, Alliance.Blue),
        new ShiftWindow( 55.0,  30.0, Alliance.Red),
    };

    // ── Public types ──────────────────────────────────────────────────────────

    public enum HubState {
        /** Hub is definitively active. */
        STRICTLY_ACTIVE,
        /** Within early/late shoot margin — treat as active but not yet strict. */
        MARGIN_ACTIVE,
        /** Hub is inactive — do not shoot. */
        INACTIVE
    }

    public static final class HubStatus {
        public final HubState state;
        /** Seconds until the next strict boundary flip. -1 if unknown. */
        public final double secsUntilNextShift;

        HubStatus(HubState state, double secsUntilNextShift) {
            this.state = state;
            this.secsUntilNextShift = secsUntilNextShift;
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Returns the full hub status for the given alliance, including state and
     * time until next shift boundary. Logs to SmartDashboard as a side effect.
     */
    public static HubStatus getHubStatus(Alliance alliance) {
        double matchTime = DriverStation.getMatchTime();

        // Outside shift period (auto, transition, endgame) — always strictly active
        if (matchTime > 130 || matchTime <= 30) {
            double secsUntilShift = (matchTime > 130) ? matchTime - 130 : Math.max(0, matchTime);
            return new HubStatus(HubState.STRICTLY_ACTIVE, secsUntilShift);
        }

        ShiftWindow[] schedule = resolveSchedule();
        if (schedule == null) {
            // No FMS data yet — treat as active, timer unknown
            return new HubStatus(HubState.STRICTLY_ACTIVE, -1);
        }

        double flooredTime = Math.floor(matchTime);

        for (ShiftWindow window : schedule) {
            double marginStart = window.high + SHOOT_EARLY_SECS;
            double marginEnd   = window.low  - SHOOT_LATE_SECS;

            if (flooredTime > marginStart || flooredTime < marginEnd) continue;

            // We're within this window's margin band
            boolean strictlyInWindow = flooredTime <= window.high && flooredTime > window.low;
            boolean isActiveAlliance = window.inactive != alliance;
            double secsUntilNextShift = flooredTime - window.low;

            if (!strictlyInWindow) {
                // In early or late margin
                return new HubStatus(HubState.MARGIN_ACTIVE, secsUntilNextShift);
            } else if (isActiveAlliance) {
                return new HubStatus(HubState.STRICTLY_ACTIVE, secsUntilNextShift);
            } else {
                return new HubStatus(HubState.INACTIVE, secsUntilNextShift);
            }
        }

        // Between windows but inside shift period — shouldn't normally happen
        return new HubStatus(HubState.STRICTLY_ACTIVE, -1);
    }

    /**
     * True when it is safe to shoot — either strictly active or within margins.
     * Also logs "Shooter/Hub Considered Active" to SmartDashboard.
     */
    public static boolean isHubShootWindowOpen(Alliance alliance) {
        HubStatus status = getHubStatus(alliance);
        boolean active = status.state != HubState.INACTIVE;
        SmartDashboard.putBoolean("Shooter/Hub Considered Active", active);
        return active;
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    private static ShiftWindow[] resolveSchedule() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) return null;

        char firstInactive = gameData.charAt(0);
        if (firstInactive == 'R') return SCHEDULE_RED_FIRST;
        if (firstInactive == 'B') return SCHEDULE_BLUE_FIRST;
        return null;
    }
}