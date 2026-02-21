// src/main/java/frc/robot/commands/Intake/IntakeArmEnableDropCommand.java
package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.Intake.IntakeArmSubsystem;

public class IntakeArmEnableDropCommand extends Command {
  private final IntakeArmSubsystem arm;
  private final Timer timer = new Timer();

  private double downDeg;
  private double enableCruiseRps;
  private double enableAccelRps2;

  private double teleopCruiseRps;
  private double teleopAccelRps2;

  public IntakeArmEnableDropCommand(IntakeArmSubsystem arm) {
    this.arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    downDeg = SmartDashboard.getNumber("IntakeArm/DownDeg", IntakeArmConstants.kPosDegA);

    enableCruiseRps = SmartDashboard.getNumber("IntakeArm/EnableCruiseRps", IntakeArmConstants.kEnableCruiseRps_Arm);
    enableAccelRps2 = SmartDashboard.getNumber("IntakeArm/EnableAccelRps2", IntakeArmConstants.kEnableAccelRps2_Arm);

    teleopCruiseRps = SmartDashboard.getNumber("IntakeArm/TeleopCruiseRps", IntakeArmConstants.kCruiseRps_Arm);
    teleopAccelRps2 = SmartDashboard.getNumber("IntakeArm/TeleopAccelRps2", IntakeArmConstants.kAccelRps2_Arm);

    // Ensure manual mode is off so MotionMagic is actually in control
    arm.disableManualControl();

    // Slow drop constraints for enable
    arm.setMotionMagicConstraintsArm(enableCruiseRps, enableAccelRps2);

    // Go to down target (-45)
    arm.setGoalDegrees(downDeg);
  }

  @Override
  public void execute() {
    // Keep commanding the down goal while this command owns the subsystem
    arm.setGoalDegrees(downDeg);
  }

  @Override
  public boolean isFinished() {
    // Finish when at goal, or timeout safety
    return arm.atGoal() || timer.hasElapsed(2.0);
  }

  @Override
  public void end(boolean interrupted) {
    // Restore teleop constraints
    arm.setMotionMagicConstraintsArm(teleopCruiseRps, teleopAccelRps2);
  }
}