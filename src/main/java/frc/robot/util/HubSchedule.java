package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
/**
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
 * Notes:
 * - isHubInactive(): strict FMS logic (only true during shifts).
 * - isHubShootWindowOpen(): adds early/late margins(based on advice form this thread: https://www.chiefdelphi.com/t/2026-game-data-and-match-time/512330/5)
 * - Outside shift windows (AUTO/TRANSITION/ENDGAME), hubs are treated as active.
 */
public final class HubSchedule {
    private HubSchedule() {}

    // Tuning knobs
    private static final double SHOOT_EARLY_SECS = 2.0; // start shooting before active
    private static final double SHOOT_LATE_SECS  = 1.9; //deal with rounding error because FMS sends it rounded to nearest integer, and we want both as 2 seconds
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

    private static ShiftWindow[] resolveSchedule() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) return null;

        char firstInactive = gameData.charAt(0);
        if (firstInactive == 'R') return SCHEDULE_RED_FIRST;
        if (firstInactive == 'B') return SCHEDULE_BLUE_FIRST;
        return null;
    }

    // public static boolean isHubInactive(Alliance alliance) {
    //     ShiftWindow[] schedule = resolveSchedule();
    //     if (schedule == null) return false;

    //     double matchTime = DriverStation.getMatchTime();
    //     for (ShiftWindow window : schedule) {
    //         if (matchTime <= window.high && matchTime > window.low) {
    //             return window.inactive == alliance;
    //         }
    //     }
    //     return false;
    // }

    /**
     * True when this alliance's hub is in a usable shoot window:
     * start shooting a bit before active, and keep going briefly after it flips inactive.
     */
    public static boolean isHubShootWindowOpen(Alliance alliance) {
        ShiftWindow[] schedule = resolveSchedule();
        if (schedule == null) return false;

        double matchTime = Math.floor(DriverStation.getMatchTime()); //while practicing at shop, we don't have FMS to round for us, so we round down ourselves to simulate real match 

            for (ShiftWindow window : schedule) {
            // Our alliance is active during this window.
            if (window.inactive != alliance) {
                double start = window.high + SHOOT_EARLY_SECS;
                double end   = window.low  - SHOOT_LATE_SECS;

                if (matchTime <= start && matchTime >= end) {
                    return true;
                }
            }
        }
        return true; // treat as active outside of shift windows, and in practice when matchtime = -1;
    }
}
