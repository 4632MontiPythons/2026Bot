package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Deploys the intake and holds it down.Finishes immediately
 */
public class DeployIntake extends Command {

    private final Intake m_intake;

    public DeployIntake(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.deploy();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}