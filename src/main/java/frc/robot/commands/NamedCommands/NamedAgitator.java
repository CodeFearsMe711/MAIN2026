package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator.AgitatorSubsystem;
import frc.robot.commands.Agitator.AgitatorCommand;

public class NamedAgitator extends Command {

  private final AgitatorCommand agitatorCommand;

  public NamedAgitator(AgitatorSubsystem agitator) {
    agitatorCommand = new AgitatorCommand(agitator, 30.0);
  }

  @Override
  public void initialize() {
    agitatorCommand.initialize();
  }

  @Override
  public void execute() {
    agitatorCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    agitatorCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return agitatorCommand.isFinished();
  }
}
