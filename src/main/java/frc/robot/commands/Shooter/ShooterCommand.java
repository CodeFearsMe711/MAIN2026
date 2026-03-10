package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
  private final ShooterSubsystem shooter;
  private final double targetRPS;

  public ShooterCommand(ShooterSubsystem shooter, double targetRPS) {
    this.shooter = shooter;
    this.targetRPS = targetRPS;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setManualRPS(targetRPS);
  }

  @Override
  public void execute() {
    shooter.setManualRPS(targetRPS);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.clearManualRPS();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}