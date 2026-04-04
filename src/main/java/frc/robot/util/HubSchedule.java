package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Tracks hub activity state throughout a match based on FRC game-specific messaging.
 * * Strict shift windows (matchTime counts down from 2:20 during TELEOP):
 * - TRANSITION: 140–130s (both active)
 * - SHIFT 1:    130–105s
 * - SHIFT 2:    105–80s
 * - SHIFT 3:     80–55s
 * - SHIFT 4:     55–30s
 * - END GAME:    30–0s   (both active)
 * * Margin: within 2.0s of a boundary, the hub is MARGIN_ACTIVE if active in either 
 * adjacent window.
 */
public final class HubSchedule {
    private HubSchedule() {}

    private static final double SHOOT_MARGIN_SECS = 2.0;

    // --- Internal Data Structures ---

    private static final class ShiftWindow {
        final double high; // matchTime start
        final double low;  // matchTime end
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

    public enum HubState {
        STRICTLY_ACTIVE,
        MARGIN_ACTIVE,
        INACTIVE
    }

    public static final class HubStatus {
        public final HubState state;
        public final double secsUntilNextBoundary;

        HubStatus(HubState state, double secsUntilNextBoundary) {
            this.state = state;
            this.secsUntilNextBoundary = secsUntilNextBoundary;
        }
    }

    // --- Public API ---

    /**
     * Returns the current hub status based on Alliance and match time.
     */
    public static HubStatus getHubStatus() {
        Alliance alliance = DriverStation.getAlliance().orElse(null);
        double matchTime = DriverStation.getMatchTime();

        // Outside of specific shift periods, both hubs are active
        if (matchTime > 130.0 || matchTime <= 30.0) {
            double secsUntilShiftPeriod = (matchTime > 130.0) ? matchTime - 130.0 : -1;
            return new HubStatus(HubState.STRICTLY_ACTIVE, secsUntilShiftPeriod);
        }

        ShiftWindow[] schedule = resolveSchedule();
        if (schedule == null) {
            return new HubStatus(HubState.STRICTLY_ACTIVE, -1);
        }

        ShiftWindow containingWindow = null;
        double nearestBoundaryDist = Double.MAX_VALUE;
        double nearestBoundary = -1;

        // Determine which window we are in and the distance to the nearest boundary
        for (ShiftWindow w : schedule) {
            double distHigh = Math.abs(matchTime - w.high);
            if (distHigh < nearestBoundaryDist) {
                nearestBoundaryDist = distHigh;
                nearestBoundary = w.high;
            }
            double distLow = Math.abs(matchTime - w.low);
            if (distLow < nearestBoundaryDist) {
                nearestBoundaryDist = distLow;
                nearestBoundary = w.low;
            }
            if (matchTime <= w.high && matchTime > w.low) {
                containingWindow = w;
            }
        }

        double secsUntilNextBoundary = (nearestBoundary >= 0) ? Math.abs(matchTime - nearestBoundary) : -1;

        // Handle Margin Logic
        if (nearestBoundaryDist <= SHOOT_MARGIN_SECS) {
            boolean activeInAnyAdjacentWindow = false;
            for (ShiftWindow w : schedule) {
                if (Math.abs(w.high - nearestBoundary) < 0.01 || Math.abs(w.low - nearestBoundary) < 0.01) {
                    if (w.inactive != alliance) {
                        activeInAnyAdjacentWindow = true;
                        break;
                    }
                }
            }
            // If we are at the edge of the 130s or 30s boundaries, we are adjacent to a "STRICTLY_ACTIVE" period
            if (Math.abs(nearestBoundary - 130.0) < 0.01 || Math.abs(nearestBoundary - 30.0) < 0.01) {
                activeInAnyAdjacentWindow = true;
            }

            return new HubStatus(
                activeInAnyAdjacentWindow ? HubState.MARGIN_ACTIVE : HubState.INACTIVE,
                secsUntilNextBoundary
            );
        }

        // Handle strict window logic
        if (containingWindow != null) {
            HubState state = (containingWindow.inactive != alliance)
                ? HubState.STRICTLY_ACTIVE
                : HubState.INACTIVE;
            return new HubStatus(state, secsUntilNextBoundary);
        }

        return new HubStatus(HubState.STRICTLY_ACTIVE, -1);
    }


    public static boolean isHubShootWindowOpen() {
        HubStatus status = getHubStatus();
        return status.state != HubState.INACTIVE;
    }

    // --- Private Helpers ---

    private static ShiftWindow[] resolveSchedule() {
        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) return null;
        char first = gameData.charAt(0);
        if (first == 'R') return SCHEDULE_RED_FIRST;
        if (first == 'B') return SCHEDULE_BLUE_FIRST;
        return null;
    }
}