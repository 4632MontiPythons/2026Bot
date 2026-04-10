package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Drive;
import frc.robot.Constants.kShooter;
import static edu.wpi.first.units.Units.*;

/*
 * Shooter subsystem with velocity closed-loop control and SysID.
 * KrakenX60 using VelocityVoltage. Debating keeping shooter warmed for whole match.
 * Will depend on whether or not we add a flywheel, the voltage we need to keep it running and how our battery is looking.
 * There is a 1:1 gear ratio
 */
public class Shooter extends SubsystemBase {

    private final TalonFX m_motor;
    private double m_lastDashRpm = -1.0;
    private double rpmAdjust = 0;
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0).withEnableFOC(false);

    // ── Cached high-frequency signals ────────────────────────────────────────
    private final StatusSignal<Angle> m_posSignal;
    private final StatusSignal<AngularVelocity> m_velSignal;
    private final StatusSignal<Voltage> m_voltSignal;
    private final StatusSignal<Current> m_currentSignal;

    // ── State ─────────────────────────────────────────────────────────────────
    private double m_targetRpm = 0.0;

    private final SysIdRoutine m_sysIdRoutine;

    // ── Motor config ──────────────────────────────────────────────────────────
    private static TalonFXConfiguration buildMotorConfig() {
        var cfg = new TalonFXConfiguration();

        cfg.CurrentLimits
                .withSupplyCurrentLimit(70.0)
                .withSupplyCurrentLowerLimit(40.0)
                .withSupplyCurrentLowerTime(1.0)
                .withStatorCurrentLimit(120.0)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(true);

        cfg.MotorOutput
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive);
        cfg.Slot0 //tuned with sysID
                .withKS(0.091)
                .withKV(0.1216)
                .withKA(0.032) //no longer accurate, but not used
                .withKP(0.185)
                .withKI(0.02)
                .withKD(0.03);
        return cfg;
    }

    // ── Constructor ───────────────────────────────────────────────────────────
    public Shooter() {
        m_motor = new TalonFX(kShooter.shooterMotorID, "SWERVE");
        m_motor.getConfigurator().apply(buildMotorConfig());

        m_posSignal = m_motor.getPosition();
        m_velSignal = m_motor.getVelocity();
        m_voltSignal = m_motor.getMotorVoltage();
        m_currentSignal = m_motor.getStatorCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(250,
                m_posSignal, m_velSignal, m_voltSignal, m_currentSignal);
        m_motor.optimizeBusUtilization();

        m_sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).per(Second),
                        Volts.of(5.0),
                        Seconds.of(10)),
                new SysIdRoutine.Mechanism(
                        volts -> m_motor.setVoltage(volts.in(Volts)),
                        log -> {
                            BaseStatusSignal.refreshAll(m_posSignal, m_velSignal, m_voltSignal, m_currentSignal);
                            log.motor("shooter")
                                    .voltage(Volts.of(m_voltSignal.getValueAsDouble()))
                                    .angularPosition(Rotations.of(m_posSignal.getValueAsDouble()))
                                    .angularVelocity(RotationsPerSecond.of(m_velSignal.getValueAsDouble()));
                        },
                        this));
        SmartDashboard.putNumber("Shooter/TestRPM", 0.0);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Set shooter speed by looking up motor shaft RPM from the interpolating distance table.
     * @param distanceMeters distance to target in meters
     */
    public void setShootingDistance(double distanceMeters) {
        double rpm = kShooter.rpmTable.get(distanceMeters);
        setRPM(rpm);
    }

    /**
     * Command a target motor shaft RPM using onboard velocity closed-loop.
     * @param rpm target motor shaft rotations per minute
     */
    public void setRPM(double rpm) {
        m_targetRpm = Math.min(rpm + rpmAdjust, kShooter.kMaxMotorRPM);
        m_motor.setControl(m_velocityRequest.withVelocity(m_targetRpm / 60.0));
    }

    // /** Stop the shooter and coast. */
    // public void stop() {
    //     m_targetRpm = 0.0;
    //     m_motor.setControl(m_stopRequest);
    // } removed because i set default command to stay at 2000 rpm

    /** @return current measured motor shaft speed in RPM */
    public double getMeasuredRPM() {
        return m_velSignal.getValueAsDouble() * 60.0;
    }

    /** @return true if shooter is within tolerance of target RPM */
    public boolean atTargetRPM(double tolerance) {
        return Math.abs(getMeasuredRPM() - m_targetRpm) < tolerance;
    }

    /** @return the last commanded target motor shaft RPM */
    public double getTargetRPM() {
        return m_targetRpm;
    }

    /** @return stator current drawn by the shooter motor in amps */
    public double getStatorCurrent() {
        return m_currentSignal.getValueAsDouble();
    }

    public void adjustRPM(double adjustment) {
        rpmAdjust += adjustment;
    }

    // ── SysID Commands ────────────────────────────────────────────────────────

    /**
     * Quasistatic SysID test. Only available outside competition mode.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        if (!Drive.comp)
            return m_sysIdRoutine.quasistatic(direction);
        return Commands.none();
    }

    /**
     * Dynamic SysID test. Only available outside competition mode.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        if (!Drive.comp)
            return m_sysIdRoutine.dynamic(direction);
        return Commands.none();
    }

    // ── Periodic ──────────────────────────────────────────────────────────────
    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(m_posSignal, m_velSignal, m_voltSignal, m_currentSignal);

        double measuredRpm = getMeasuredRPM();
        SmartDashboard.putNumber("Shooter/TargetRPM", m_targetRpm);
        SmartDashboard.putNumber("Shooter/MeasuredRPM", measuredRpm);
        SmartDashboard.putNumber("Shooter/StatorCurrent", getStatorCurrent());
        if (!Drive.comp && getCurrentCommand() == null) {
            double dashRpm = SmartDashboard.getNumber("Shooter/TestRPM", 0.0);
            if (dashRpm != m_lastDashRpm) {
                m_lastDashRpm = dashRpm;
                if (dashRpm > 0.0) {
                    setRPM(dashRpm);
                }
            }
        }
    }
}