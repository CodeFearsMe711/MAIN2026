package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.Intake.IntakeArmSubsystem;

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
    return arm.atGoalRangeDeg(targetDeg, IntakeArmConstants.kToleranceDeg);
  }

  @Override
  public void end(boolean interrupted) {
    arm.holdCurrentPosition();
  }
}