package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kFeeder;
/**
 * Redline motor ran by spark max. Simple setup.
 */
public class Feeder extends SubsystemBase {

    private final SparkMax m_motor;

    // ── Constructor ────────────────────────────────────────────────────────────
    public Feeder() {
        m_motor = new SparkMax(kFeeder.feederMotorID, MotorType.kBrushed);
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kBrake)
            .inverted(kFeeder.inverted)
            .openLoopRampRate(0.1);
        config.smartCurrentLimit(25);
        config.inverted(kFeeder.inverted);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // ── Public API ─────────────────────────────────────────────────────────────

    /** Run the feeder at default feed speed. */
    public void feed() {
        setMotor(kFeeder.feedSpeed);
    }

    /**
     * Run feeder at a custom speed.
     * @param speed -1.0 to 1.0
     */
    public void setSpeed(double speed) {
        setMotor(speed);
    }

    /** Reverse the feeder. */
    public void reverse() {
        setMotor(kFeeder.reverseSpeed);
    }

    /** Stop the feeder. */
    public void stop() {
        setMotor(0.0);
    }
    public boolean isFeeding(){
        return m_motor.get()>0.2;
    }


    // ── Internal ───────────────────────────────────────────────────────────────

    private void setMotor(double speed) {
        m_motor.set(speed);
    }
    public void test(){
        setMotor(.50);
    }

    // ── Periodic ───────────────────────────────────────────────────────────────
    @Override
    public void periodic() {}
}