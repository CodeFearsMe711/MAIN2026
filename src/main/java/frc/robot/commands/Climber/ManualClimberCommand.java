package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class ManualClimberCommand extends Command {
  public enum ClimberDirection {
    UP,
    DOWN
  }

  private final ClimberSubsystem climber;
  private final ClimberDirection direction;

  public ManualClimberCommand(ClimberSubsystem climber, ClimberDirection direction) {
    this.climber = climber;
    this.direction = direction;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (direction == ClimberDirection.UP) {
      climber.driveUpManual();
    } else {
      climber.driveDownManual();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (direction == ClimberDirection.UP) {
      if (climber.getDegrees() > ClimberConstants.kMinDeg + 1.0) {
        climber.holdCurrentPosition();
      } else {
        climber.stopMotors();
      }
    } else {
      climber.stopMotors();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}