package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.commands.Shooter.ShooterCommand;

public class NamedShooter extends Command {

  private final ShooterCommand shooterCommand;

  public NamedShooter(ShooterSubsystem shooter) {
    shooterCommand = new ShooterCommand(shooter, 50.0); // RPS
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
