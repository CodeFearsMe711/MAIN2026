package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
  private final ShooterSubsystem m_subsystem;
  private final double m_targetRPS;

  public ShooterCommand(ShooterSubsystem subsystem, double targetRPS) {
    m_subsystem = subsystem;
    m_targetRPS = targetRPS;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    m_subsystem.updateVisionSpeed(m_targetRPS);
  }

  @Override
  public void execute() {
    m_subsystem.updateVisionSpeed(m_targetRPS);
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}