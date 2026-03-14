package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive;
import frc.robot.Constants.kFeeder;

public class Feeder extends SubsystemBase {

    private final SparkMax m_motor;
    private double m_currentSpeed = 0.0;

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

    /** @return output current drawn by the feeder motor in amps */
    public double getOutputCurrent() {
        return m_motor.getOutputCurrent();
    }

    // ── Internal ───────────────────────────────────────────────────────────────

    private void setMotor(double speed) {
        m_currentSpeed = speed;
        m_motor.set(speed);
    }

    // ── Periodic ───────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder/SetSpeed",      m_currentSpeed);
        SmartDashboard.putNumber("Feeder/OutputCurrent", getOutputCurrent());

        if (!Drive.comp && getCurrentCommand() == null) {
            double dashSpeed = SmartDashboard.getNumber("Feeder/TestSpeed", 0.0);
            SmartDashboard.putNumber("Feeder/TestSpeed", dashSpeed);
            if (dashSpeed != 0.0) {
                setSpeed(dashSpeed);
            } else {
                stop();
            }
        }
    }
}