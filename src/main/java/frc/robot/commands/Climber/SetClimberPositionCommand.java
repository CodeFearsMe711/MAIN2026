package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class SetClimberPositionCommand extends Command {
  private final ClimberSubsystem climber;
  private static final double kTargetDegrees = 1100.0;
  private static final double kToleranceDeg = 5.0;

  public SetClimberPositionCommand(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    climber.setTargetDegrees(kTargetDegrees);
  }

  @Override
  public boolean isFinished() {
    return climber.isAtTargetDegrees(kTargetDegrees, kToleranceDeg);
  }

  @Override
  public void end(boolean interrupted) {
    climber.holdCurrentPosition();
  }
}