package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

public class ShooterCommand extends Command {
   private final ShooterSubsystem m_subsystem;
  private final double m_targetRPS;

  
  public ShooterCommand(ShooterSubsystem subsystem, double targetRPS) {
    m_subsystem = subsystem;
    m_targetRPS = targetRPS;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Ensure we start at the requested speed immediately
    m_subsystem.setRPS(m_targetRPS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Re-apply the target continuously to ensure the controller keeps the speed
    m_subsystem.setRPS(m_targetRPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motor when the command finishes or is interrupted
    m_subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command runs until explicitly canceled; change if you want a timed/conditional command
    return false;
  }
}
