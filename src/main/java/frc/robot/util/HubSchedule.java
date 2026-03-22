package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Encodes the hub active/inactive shift schedule and answers queries about
 * whether a given alliance's hub is currently inactive or imminently going
 * inactive.
 *
 * Shift schedule -------------------------------------------------------
 * Game data encodes which alliance's hub is FIRST inactive. All times are
 * match-timer countdown values (seconds remaining).
 *
 *   RED_FIRST_INACTIVE (gameData.charAt(0) == 'R'):
 *     Shift 1 (2:10–1:45)  Red INACTIVE, Blue active
 *     Shift 2 (1:45–1:20)  Red active,   Blue INACTIVE
 *     Shift 3 (1:20–0:55)  Red INACTIVE, Blue active
 *     Shift 4 (0:55–0:30)  Red active,   Blue INACTIVE
 *
 *   BLUE_FIRST_INACTIVE (gameData.charAt(0) == 'B'):
 *     Shift 1 (2:10–1:45)  Red active,   Blue INACTIVE
 *     Shift 2 (1:45–1:20)  Red INACTIVE, Blue active
 *     Shift 3 (1:20–0:55)  Red active,   Blue INACTIVE
 *     Shift 4 (0:55–0:30)  Red INACTIVE, Blue active
 *
 * Outside these windows (AUTO, TRANSITION, END GAME) both hubs are active.
 */
public final class HubSchedule {

    private HubSchedule() {} // utility class — no instances

    // ── Internal window record ─────────────────────────────────────────────

    private static final class ShiftWindow {
        final double high;
        final double low;
        final Alliance inactive;

        ShiftWindow(double high, double low, Alliance inactive) {
            this.high     = high;
            this.low      = low;
            this.inactive = inactive;
        }
    }

    // ── Schedules ──────────────────────────────────────────────────────────

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

    // ── Private helpers ────────────────────────────────────────────────────

    /**
     * Returns the correct schedule based on game-specific message, or
     * {@code null} if the message is absent or unrecognised.
     */
    private static ShiftWindow[] resolveSchedule() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) return null;
        char firstInactive = gameData.charAt(0);
        if (firstInactive == 'R') return SCHEDULE_RED_FIRST;
        if (firstInactive == 'B') return SCHEDULE_BLUE_FIRST;
        return null;
    }

    // ── Public API ─────────────────────────────────────────────────────────

    /**
     * Returns {@code true} if {@code alliance}'s hub is inactive right now
     * according to the shift schedule.
     *
     * @param alliance alliance to check
     * @return {@code true} if the hub is currently inactive
     */
    public static boolean isHubInactive(Alliance alliance) {
        ShiftWindow[] schedule = resolveSchedule();
        if (schedule == null) return false;

        double matchTime = DriverStation.getMatchTime();
        for (ShiftWindow window : schedule) {
            if (matchTime <= window.high && matchTime > window.low) {
                return window.inactive == alliance;
            }
        }
        return false;
    }

    /**
     * Returns {@code true} if {@code alliance}'s hub is inactive now <em>or</em>
     * will become inactive within {@code bufferSecs} seconds.
     *
     * @param alliance   alliance to check
     * @param bufferSecs look-ahead buffer (seconds)
     * @return {@code true} if the hub is inactive or imminently going inactive
     */
    public static boolean isHubImminentlyInactive(Alliance alliance, double bufferSecs) {
        ShiftWindow[] schedule = resolveSchedule();
        if (schedule == null) return false;

        double matchTime = DriverStation.getMatchTime();
        for (ShiftWindow window : schedule) {
            // Currently inside this window and it's our alliance's inactive shift
            if (matchTime <= window.high && matchTime > window.low) {
                return window.inactive == alliance;
            }
            // Just outside the top of a window affecting our alliance — within buffer
            if (window.inactive == alliance
                    && matchTime > window.high
                    && matchTime <= window.high + bufferSecs) {
                return true;
            }
        }
        return false;
    }
}