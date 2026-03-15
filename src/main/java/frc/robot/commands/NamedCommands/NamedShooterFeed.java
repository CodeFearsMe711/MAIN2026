package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterFeederSubsytem;

public class NamedShooterFeed extends Command {
  private final ShooterFeederSubsytem feeder;

  public NamedShooterFeed(ShooterFeederSubsytem feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
  }

  @Override
  public void initialize() {
    feeder.setRPS(40.0);
  }

  @Override
  public void execute() {
    feeder.setRPS(40.0);
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
