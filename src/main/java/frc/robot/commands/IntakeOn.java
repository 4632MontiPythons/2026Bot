package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
/* Runs intake, be careful, because it will run even if not deployed*/
public class IntakeOn extends Command {
    private final Intake m_intake;
    public IntakeOn(Intake intake){
        m_intake = intake;
    }
    @Override
    public void execute() {
            m_intake.runIntake();
    }
    public void end(boolean interrupted){
        m_intake.stopIntake();
    }
}
