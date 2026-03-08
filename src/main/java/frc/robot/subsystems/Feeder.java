package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.Constants.kFeeder;

public class Feeder extends SubsystemBase {

    private final SparkMax m_motor;

    // ── Jam recovery state machine ─────────────────────────────────────────────
    private enum FeedState { FEEDING, JAMMED_REVERSING, JAMMED_RETRYING }
    private FeedState m_state = FeedState.FEEDING;
    private final Timer m_jamTimer = new Timer();

    private double m_currentSpeed = 0.0;
    private boolean m_feedRequested = false;

    // ── Constructor ────────────────────────────────────────────────────────────
    public Feeder() {
        m_motor = new SparkMax(kFeeder.feederMotorID, MotorType.kBrushed);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .inverted(kFeeder.inverted)
            .openLoopRampRate(0.1);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ── Public API ─────────────────────────────────────────────────────────────

    /** Run the feeder at default feed speed. Jam recovery runs automatically. */
    public void feed() {
        m_feedRequested = true;
        // Only push the motor directly if we're not mid-recovery;
        // periodic() owns the output while jam handling is active.
        if (m_state == FeedState.FEEDING) {
            setMotor(kFeeder.feedSpeed);
        }
    }

    /**
     * Run feeder at a custom speed (bypasses jam recovery — use for manual control).
     * @param speed -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        m_feedRequested = false;
        m_state = FeedState.FEEDING;
        setMotor(speed);
    }

    /** Manual reverse to unjam (bypasses auto recovery). */
    public void reverse() {
        m_feedRequested = false;
        m_state = FeedState.FEEDING;
        setMotor(kFeeder.reverseSpeed);
    }

    /** Stop the feeder and reset jam recovery state. */
    public void stop() {
        m_feedRequested = false;
        m_state = FeedState.FEEDING;
        setMotor(0.0);
    }

    /** @return output current drawn by the feeder motor in amps */
    public double getOutputCurrent() {
        return m_motor.getOutputCurrent();
    }

    /** @return true if currently in jam recovery */
    public boolean isJammed() {
        return m_state != FeedState.FEEDING;
    }

    // ── Internal helpers ───────────────────────────────────────────────────────

    private void setMotor(double speed) {
        m_currentSpeed = speed;
        m_motor.set(speed);
    }

    private boolean currentOverThreshold() {
        return getOutputCurrent() > kFeeder.jamCurrentThreshold;
    }

    // ── Periodic ───────────────────────────────────────────────────────────────
    @Override
    public void periodic() {

        // ── Jam recovery state machine (only active while feed is requested) ──
        if (m_feedRequested) {
            switch (m_state) {

                case FEEDING:
                    if (currentOverThreshold()) {
                        // Jam detected — start reversing
                        m_state = FeedState.JAMMED_REVERSING;
                        m_jamTimer.restart();
                        setMotor(kFeeder.reverseSpeed);
                    }
                    break;

                case JAMMED_REVERSING:
                    if (m_jamTimer.hasElapsed(kFeeder.reverseTimeSec)) {
                        // Reverse done — try feeding again
                        m_state = FeedState.JAMMED_RETRYING;
                        m_jamTimer.restart();
                        setMotor(kFeeder.feedSpeed);
                    }
                    break;

                case JAMMED_RETRYING:
                    if (!currentOverThreshold()) {
                        // Jam cleared — back to normal
                        m_state = FeedState.FEEDING;
                    } else if (m_jamTimer.hasElapsed(kFeeder.retryTimeSec)) {
                        // Still jammed after retry — reverse again
                        m_state = FeedState.JAMMED_REVERSING;
                        m_jamTimer.restart();
                        setMotor(kFeeder.reverseSpeed);
                    }
                    break;
            }
        }

        // ── Dashboard ──────────────────────────────────────────────────────────
        SmartDashboard.putNumber("Feeder/Speed",         m_currentSpeed);
        SmartDashboard.putNumber("Feeder/OutputCurrent", getOutputCurrent());
        SmartDashboard.putString("Feeder/State",         m_state.toString());

        if (!Drive.comp) {
            SmartDashboard.putNumber("Feeder/TestSpeed",
                SmartDashboard.getNumber("Feeder/TestSpeed", 0.0));
            double dashSpeed = SmartDashboard.getNumber("Feeder/TestSpeed", 0.0);
            if (dashSpeed != 0.0) {
                setSpeed(dashSpeed);
            } else {
                stop();
            }
        }
    }
}