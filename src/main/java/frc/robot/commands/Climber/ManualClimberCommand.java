package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class ManualClimberCommand extends Command {
  public enum Direction {
    UP,
    DOWN
  }

  private final ClimberSubsystem climber;
  private final Direction direction;

  public ManualClimberCommand(ClimberSubsystem climber, Direction direction) {
    this.climber = climber;
    this.direction = direction;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    if (direction == Direction.UP) {
      climber.driveUp();
    } else {
      climber.driveDown();
    }
  }

  @Override
  public void execute() {
    if (direction == Direction.UP) {
      climber.driveUp();
    } else {
      climber.driveDown();
    }
  }

  @Override
  public void end(boolean interrupted) {
    climber.holdCurrentPosition();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}