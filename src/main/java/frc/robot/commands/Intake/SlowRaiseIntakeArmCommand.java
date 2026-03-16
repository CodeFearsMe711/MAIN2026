package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeArmSubsystem;

/**
 * Command that exclusively drives the intake arm upward slowly.
 * While this command is scheduled it disables the subsystem's normal
 * position-hold and drives the arm at a small constant RPS. The command
 * is set non-interruptible so other commands won't preempt it unless
 * explicitly forced.
 */
public class SlowRaiseIntakeArmCommand extends Command {

  private final IntakeArmSubsystem arm;
  private final double speedRps;

  /**
   * Create the command.
   * @param arm the intake arm subsystem
   * @param speedRps positive arm RPS to drive upward slowly (e.g. 0.1)
   */
  public SlowRaiseIntakeArmCommand(IntakeArmSubsystem arm, double speedRps) {
    this.arm = arm;
    this.speedRps = Math.abs(speedRps);
    addRequirements(arm);
  // This command style uses Command instead of CommandBase
  // No longer setting interruptible as per new command style
  }

  @Override
  public void initialize() {
    // Enable manual velocity control upward
    arm.enableManualArmRPS(speedRps);
  }

  @Override
  public void execute() {
    // nothing to do each loop; subsystem periodic will apply the velocity
  }

  @Override
  public void end(boolean interrupted) {
    // Stop manual control and set the current position as the goal so Motion Magic holds
    arm.disableManualControl();
    arm.setGoalDegrees(arm.getDegrees());
  }

  @Override
  public boolean isFinished() {
    return false; // run until cancelled
  }
}
