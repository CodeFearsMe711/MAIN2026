package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator.AgitatorSubsystem;

public class NamedAgitator extends Command {
  private final AgitatorSubsystem agitator;

  public NamedAgitator(AgitatorSubsystem agitator) {
    this.agitator = agitator;
    addRequirements(agitator);
  }

  @Override
  public void initialize() {
    agitator.setRPS(30.0);
  }

  @Override
  public void execute() {
    agitator.setRPS(30.0);
  }

  @Override
  public void end(boolean interrupted) {
    agitator.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}