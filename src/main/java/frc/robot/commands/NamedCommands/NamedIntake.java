package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.commands.Intake.IntakeCommand;

public class NamedIntake extends Command {

  private final IntakeCommand intakeCommand;

  public NamedIntake(IntakeSubsystem intake) {
    intakeCommand = new IntakeCommand(intake, 35.0);
  }

  @Override
  public void initialize() {
    intakeCommand.initialize();
  }

  @Override
  public void execute() {
    intakeCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    intakeCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return intakeCommand.isFinished();
  }
}
