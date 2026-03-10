package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Shooter.ShooterCommand;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class NamedShooter extends Command {
  private final ShooterCommand shooterCommand;

  public NamedShooter(ShooterSubsystem shooter) {
    shooterCommand = new ShooterCommand(shooter, 55.0);
  }

  @Override
  public void initialize() {
    shooterCommand.initialize();
  }

  @Override
  public void execute() {
    shooterCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    shooterCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return shooterCommand.isFinished();
  }
}