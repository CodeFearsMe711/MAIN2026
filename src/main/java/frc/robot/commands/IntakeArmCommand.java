package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmCommand extends Command {
  private final IntakeArmSubsystem arm;
  private final double targetDeg;

  public IntakeArmCommand(IntakeArmSubsystem arm, double targetDeg) {
    this.arm = arm;
    this.targetDeg = targetDeg;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setGoalDegrees(targetDeg);
  }

  @Override
  public boolean isFinished() {
    return arm.atGoal();
  }
}
