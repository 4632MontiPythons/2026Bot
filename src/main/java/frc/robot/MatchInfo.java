package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.Optional;

/**
 * Centralized holder for FMS / alliance data.
 * - Call ensureInitialized() periodically to attempt to cache FMS data when it becomes available.
 * - Other code can call isFmsDataValid() and isOwnAllianceInactive() without duplicating DriverStation logic.
 */
public final class MatchInfo {
    private static MatchInfo s_instance;

    private volatile boolean m_fmsDataValid = false;
    private volatile boolean m_ownAllianceInactive = false;
    private volatile DriverStation.Alliance m_ownAlliance = null;

    private MatchInfo() {}

    public static synchronized MatchInfo getInstance() {
        if (s_instance == null) s_instance = new MatchInfo();
        return s_instance;
    }

    /**
     * Try to read FMS game data and alliance. Returns true if data is now available and cached.
     * Non-blocking; safe to call from robotPeriodic.
     */
    public synchronized boolean ensureInitialized() {
        if (m_fmsDataValid) return true;

        String gameData = DriverStation.getGameSpecificMessage(); //gets first alliance active shift
        var alliance = DriverStation.getAlliance();

        if (gameData != null && !gameData.isEmpty() && alliance.isPresent()) {
            char inactiveFirst = gameData.charAt(0);
            m_ownAlliance = alliance.get();
            char ownAllianceChar = (m_ownAlliance == DriverStation.Alliance.Red) ? 'R' : 'B';
            m_ownAllianceInactive = (ownAllianceChar == inactiveFirst);
            m_fmsDataValid = true;
            return true;
        }

        return false;
    }

    public boolean isFmsDataValid() {
        return m_fmsDataValid;
    }

    /** True when the "inactive" alliance from FMS matches our alliance. */
    public boolean isOwnAllianceInactive() {
        return m_ownAllianceInactive;
    }

    public Optional<DriverStation.Alliance> getOwnAlliance() {
        return Optional.ofNullable(m_ownAlliance);
    }

    /** Clear cached values so next ensureInitialized() will attempt to read FMS data again. */
    public synchronized void reset() {
        m_fmsDataValid = false;
        m_ownAllianceInactive = false;
        m_ownAlliance = null;
    }
}
