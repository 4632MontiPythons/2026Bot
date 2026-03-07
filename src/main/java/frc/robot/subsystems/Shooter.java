package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Drive;
import frc.robot.Constants.kShooter;

import static edu.wpi.first.units.Units.*;
/* 
 * Shooter subsystem with velocity closed-loop control and SysID.
 * Something to note is that rpms  are in motor shaft rpm, not flywheel rpm
 * There is a 4:1 gear reduction between the motor and flywheel, so 4000 motor rpm corresponds to 1000 flywheel rpm.
 * i decided against converting just to keep it consistent everywhere.
 */
public class Shooter extends SubsystemBase {

    private final TalonFX m_motor;

    private final VelocityVoltage m_velocityRequest =
            new VelocityVoltage(0).withSlot(0).withEnableFOC(false); //too poor lol
    private final NeutralOut m_stopRequest = new NeutralOut();

    // ── State ─────────────────────────────────────────────────────────────────
    private double m_targetRpm = 0.0;
    private boolean m_isStopped = true;

    private final SysIdRoutine m_sysIdRoutine;

    // ── Motor controller configuration ───────────────────────────────────────
    private static TalonFXConfiguration buildMotorConfig() {
        var cfg = new TalonFXConfiguration();

        // --- Current limits ---
        cfg.CurrentLimits
            // Supply (from battery) — protects breakers and stops brownouts
            .withSupplyCurrentLimit(60.0)          // burst limit, can be sustained for 1 sec
            .withSupplyCurrentLowerLimit(45.0)     // lower limit, after 1 sec
            .withSupplyCurrentLowerTime(1)       // allow burst for 1 s then drop to 45 A
            .withSupplyCurrentLimitEnable(true)
            // Stator (in motor) — protects the motor itself
            .withStatorCurrentLimit(120.0)
            .withStatorCurrentLimitEnable(true);

        cfg.MotorOutput
            .withNeutralMode(NeutralModeValue.Coast)  // shooter wheels should coast when idle
            .withInverted(InvertedValue.CounterClockwise_Positive);

        // Units: rotations/second for velocity, volts for kS/kV
        // todo: Run SysID to get real kS and kV; these are placeholder values.
        cfg.Slot0
            .withKS(0.20)   // static friction (V) — from SysID
            .withKV(0.115)  // velocity gain (V per rot/s) — from SysID; ~1/kRPM*60
            .withKA(0.01)   // acceleration gain (V per rot/s²) — usually small for flywheel
            .withKP(0.15)   // proportional (V per rot/s error)
            .withKI(0.0)
            .withKD(0.0);

        // Ramp up voltage limit to avoid stressing the flywheel and tripping current limits.
        cfg.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.05); // 50 ms ramp
        return cfg;
    }

    // ── Constructor ───────────────────────────────────────────────────────────
    public Shooter() {
        m_motor = new TalonFX(kShooter.shooterMotorID, "SWERVE");
        m_motor.getConfigurator().apply(buildMotorConfig());

        // SysID: ramp voltage up and record velocity to extract kS and kV
		if(!Drive.comp){
        m_sysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),  // ramp rate: 0.5 V/s
                Volts.of(7.0),              //step voltage for kA
                Seconds.of(12.5)            // timeout
            ),
            new SysIdRoutine.Mechanism(
                // Drive: apply raw voltage
                volts -> m_motor.setVoltage(volts.in(Volts)),
                // Log: record position and velocity
                log -> {
                    var posSignal = m_motor.getPosition();
                    var velSignal = m_motor.getVelocity();
                    var voltSignal = m_motor.getMotorVoltage();
                    log.motor("shooter-flywheel")
                        .voltage(Volts.of(voltSignal.getValueAsDouble()))
                        .angularPosition(Rotations.of(posSignal.getValueAsDouble()))
                        .angularVelocity(RotationsPerSecond.of(velSignal.getValueAsDouble()));
                },
                this
            )
        );
		}
    }

    // ── Public API, call these in commands ────────────────────────────────────────────────────────────

    /**
     * Set shooter speed by looking up RPM from the interpolating distance table.
     * @param distanceMeters distance to target in meters
     */
    public void setShootingDistance(double distanceMeters) {
        double rpm = kShooter.rpmTable.get(distanceMeters);
        setRPM(rpm);
    }

    /**
     * Command a target shooter speed in RPM using onboard velocity closed-loop.
     * @param rpm target rotations per minute
     */
    public void setRPM(double rpm) {
        m_targetRpm = rpm;
        m_isStopped = false;
        // TalonFX velocity PID operates in rotations/second
        double rps = rpm / 60.0;
        m_motor.setControl(m_velocityRequest.withVelocity(rps));
    }

    /** Stop the shooter and coast */
    public void stop() {
        m_targetRpm = 0.0;
        m_isStopped = true;
        m_motor.setControl(m_stopRequest);
    }

    /** @return current measured shooter speed in RPM */
    public double getMeasuredRPM() {
        return m_motor.getVelocity().getValueAsDouble() * 60.0;
    }

    /** @return true if shooter is within ±tolerance RPM of target */
    public boolean atTargetRPM() {
        return !m_isStopped && Math.abs(getMeasuredRPM() - m_targetRpm) < kShooter.rpmTolerance;
    }

    /** @return the last commanded target RPM */
    public double getTargetRPM() { return m_targetRpm; }

    // ── SysID Commands ────────────────────────────────────────────────────────

    /**
     * Quasistatic SysID test — slowly ramps voltage to measure kS and kV.
     * Bind to a button; hold until routine completes.
     * @param direction SysIdRoutine.Direction.kForward or kReverse
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        if(!Drive.comp) return m_sysIdRoutine.quasistatic(direction);
        return null;
    }

    /**
     * Dynamic SysID test — applies a voltage step to measure kA.
     * @param direction SysIdRoutine.Direction.kForward or kReverse
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        if(!Drive.comp) return m_sysIdRoutine.dynamic(direction);
        return null;
    }

    // ── Periodic ──────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        double measuredRpm = getMeasuredRPM();
        SmartDashboard.putNumber("Shooter/TargetRPM",   m_targetRpm);
        SmartDashboard.putNumber("Shooter/MeasuredRPM", measuredRpm);
	// ── Test mode: allow RPM override from dashboard ──────────────────────
    if (!Drive.comp) {
        // putNumber only sets the default the first time; after that the
        // driver-station widget value is used when the entry already exists.
        SmartDashboard.putNumber("Shooter/TestRPM", SmartDashboard.getNumber("Shooter/TestRPM", m_targetRpm));
        double dashRpm = SmartDashboard.getNumber("Shooter/TestRPM", 0.0);
        if (dashRpm > 0.0) {
            setRPM(dashRpm);
        } else {
            stop();
        }
    }
	}
}