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
import static frc.robot.Constants.kIntake;
public class Intake extends SubsystemBase {

    // -- Hardware --
    private final SparkMax motor;
    private final DoubleSolenoid deploySolenoidA;
    private final DoubleSolenoid deploySolenoidB;

    public Intake() {
        motor = new SparkMax(kIntake.intakeMotorID, MotorType.kBrushed);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        deploySolenoidA = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, kIntake.solenoidL_Forward, kIntake.solenoidL_Reverse);
        deploySolenoidB = new DoubleSolenoid(
            PneumaticsModuleType.CTREPCM, kIntake.solenoidR_Forward, kIntake.solenoidR_Reverse);
    }

    // -- Deployment --

    public void deploy() {
        deploySolenoidA.set(Value.kForward);
        deploySolenoidB.set(Value.kForward);
    }

    public void retract() {
        deploySolenoidA.set(Value.kReverse);
        deploySolenoidB.set(Value.kReverse);
    }

    public boolean isDeployed() {
        return deploySolenoidA.get() == Value.kForward;
    }

    // -- Motor --

    public void runIntake() {
        motor.set(kIntake.intakeSpeed);
    }

    public void stopIntake() {
        motor.set(0.0);
    }

    @Override
    public void periodic() {}
}