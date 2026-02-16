package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterFeederSubsytem;
import frc.robot.commands.Shooter.ShooterFeederCommand;

public class NamedShooterFeed extends Command {

  private final ShooterFeederCommand feedCommand;

  public NamedShooterFeed(ShooterFeederSubsytem feeder) {
    feedCommand = new ShooterFeederCommand(feeder, 40.0);
  }

  @Override
  public void initialize() {
    feedCommand.initialize();
  }

  @Override
  public void execute() {
    feedCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    feedCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return feedCommand.isFinished();
  }
}
