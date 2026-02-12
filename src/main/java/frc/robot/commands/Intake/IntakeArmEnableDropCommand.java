package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.Intake.IntakeArmSubsystem;

public class IntakeArmEnableDropCommand extends Command {
  private final IntakeArmSubsystem arm;

  public IntakeArmEnableDropCommand(IntakeArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    double cruise = SmartDashboard.getNumber(
        "IntakeArm/EnableCruiseRps",
        IntakeArmConstants.kEnableCruiseRps_Arm);

    double accel = SmartDashboard.getNumber(
        "IntakeArm/EnableAccelRps2",
        IntakeArmConstants.kEnableAccelRps2_Arm);

    arm.setMotionMagicConstraintsArm(cruise, accel);

    double downDeg = SmartDashboard.getNumber(
        "IntakeArm/DownDeg",
        IntakeArmConstants.kPosDegA);

    arm.setGoalDegrees(downDeg);
  }

  @Override
  public boolean isFinished() {
    return arm.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    // Restore teleop constraints (reads dashboard so you can tune)
    double cruise = SmartDashboard.getNumber(
        "IntakeArm/TeleopCruiseRps",
        IntakeArmConstants.kCruiseRps_Arm);

    double accel = SmartDashboard.getNumber(
        "IntakeArm/TeleopAccelRps2",
        IntakeArmConstants.kAccelRps2_Arm);

    arm.setMotionMagicConstraintsArm(cruise, accel);
  }
}
