package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kIntake;

import static frc.robot.Constants.kIntake;
/**
 * It deploys immediately on auto enabled.
 * 
  */
public class Intake extends SubsystemBase {

    // ── Hardware ───────────────────────────────────────────────────────────────
    private final SparkMax motor;
    private final DoubleSolenoid solenoid;
    public Intake() {
        motor = new SparkMax(kIntake.intakeMotorID, MotorType.kBrushed);
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(kIntake.motorCurrentLimit);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        solenoid = new DoubleSolenoid(kIntake.PCM_CAN_ID,
            PneumaticsModuleType.CTREPCM, kIntake.solenoid_Forward, kIntake.solenoid_Reverse);
    }

    // ── Deployment ─────────────────────────────────────────────────────────────

    public void deploy() {
        solenoid.set(Value.kForward);
    }

    public void retract() {
        solenoid.set(Value.kReverse);
    }

    /**
     * Returns whether the intake was last commanded to deploy.
     */
    public boolean isDeployed() {
        return solenoid.get() == Value.kForward;
    }

    /** Toggles intake deployment based on last commanded state. */
    public void toggleIntake() {
        if (isDeployed()) retract(); else deploy();
    }

    // ── Motor ──────────────────────────────────────────────────────────────────

    public void runIntake() {
        motor.set(kIntake.intakeSpeed);
    }
    public void reverseIntake(){
        motor.set(-kIntake.intakeSpeed);
    }

    public void stopIntake() {
        motor.set(0.0);
    }

    @Override
    public void periodic() {

    }
}