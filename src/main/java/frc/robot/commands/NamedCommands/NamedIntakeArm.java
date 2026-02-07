package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeArmSubsystem;
import frc.robot.commands.Intake.IntakeArmCommand;

public class NamedIntakeArm extends Command {

  private final IntakeArmCommand armCommand;

  public NamedIntakeArm(IntakeArmSubsystem arm) {
    armCommand = new IntakeArmCommand(arm, 90.0); // degrees
  }

  @Override
  public void initialize() {
    armCommand.initialize();
  }

  @Override
  public void execute() {
    armCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    armCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return armCommand.isFinished();
  }
}
