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
 * We assume it always starts retracted which makes sense cause otherwise we'd violate the robot perimeter.
 * 
  */
public class Intake extends SubsystemBase {

    // ── Hardware ───────────────────────────────────────────────────────────────
    // private final SparkMax motor;
    private final DoubleSolenoid deploySolenoidA;

    public Intake() {
        // motor = new SparkMax(kIntake.intakeMotorID, MotorType.kBrushed);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(kIntake.motorCurrentLimit);

        // motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        deploySolenoidA = new DoubleSolenoid(kIntake.PCM_CAN_ID,
            PneumaticsModuleType.CTREPCM, kIntake.solenoidL_Forward, kIntake.solenoidL_Reverse);
    }

    // ── Deployment ─────────────────────────────────────────────────────────────

    public void deploy() {
        deploySolenoidA.set(Value.kForward);
        // deploySolenoidB.set(Value.kForward);
    }

    public void retract() {
        deploySolenoidA.set(Value.kReverse);
        // deploySolenoidB.set(Value.kReverse);
    }

    /**
     * Returns whether the intake was last commanded to deploy.
     */
    public boolean isDeployed() {
        return deploySolenoidA.get() == Value.kForward;
    }

    /** Toggles intake deployment based on last commanded state. */
    public void toggleIntake() {
        if (isDeployed()) retract(); else deploy();
    }

    // ── Motor ──────────────────────────────────────────────────────────────────

    public void runIntake() {
        // motor.set(kIntake.intakeSpeed);
    }

    public void stopIntake() {
        // motor.set(0.0);
    }

    @Override
    public void periodic() {}
}