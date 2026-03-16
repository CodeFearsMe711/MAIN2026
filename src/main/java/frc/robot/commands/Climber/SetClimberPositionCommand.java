package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class SetClimberPositionCommand extends Command {
  private final ClimberSubsystem climber;
  private final double targetDegrees;
  private static final double kToleranceDeg = 2.0;

  public SetClimberPositionCommand(ClimberSubsystem climber, double targetDegrees) {
    this.climber = climber;
    this.targetDegrees = targetDegrees;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setTargetDegrees(targetDegrees);
  }

  @Override
  public boolean isFinished() {
    return climber.isAtTargetDegrees(targetDegrees, kToleranceDeg);
  }

  @Override
  public void end(boolean interrupted) {
    climber.holdCurrentPosition();
  }
}