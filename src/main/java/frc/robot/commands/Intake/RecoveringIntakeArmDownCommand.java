package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.Agitator.AgitatorSubsystem;
import frc.robot.subsystems.Intake.IntakeArmSubsystem;
import frc.robot.subsystems.Shooter.ShooterFeederSubsytem;

public class RecoveringIntakeArmDownCommand extends Command {
  private enum State {
    ATTEMPT,
    CLEAR,
    RETRY,
    DONE
  }

  private final IntakeArmSubsystem arm;
  private final ShooterFeederSubsytem feeder;
  private final AgitatorSubsystem agitator;
  private final Timer stateTimer = new Timer();

  private State state = State.ATTEMPT;
  private double downDeg;
  private double tolDeg;
  private double attemptTimeoutSec;
  private double clearTimeoutSec;
  private double feederReverseRps;
  private double agitatorReverseRps;
  private boolean enableRecovery;

  public RecoveringIntakeArmDownCommand(
      IntakeArmSubsystem arm,
      ShooterFeederSubsytem feeder,
      AgitatorSubsystem agitator) {
    this.arm = arm;
    this.feeder = feeder;
    this.agitator = agitator;
    addRequirements(arm, feeder, agitator);
  }

  @Override
  public void initialize() {
    downDeg = SmartDashboard.getNumber("IntakeArm/DownDeg", IntakeArmConstants.kPosDegA);
    tolDeg = SmartDashboard.getNumber("IntakeArm/CompleteTolDeg", IntakeArmConstants.kToleranceDeg);
    attemptTimeoutSec = SmartDashboard.getNumber("IntakeArm/AutoAttemptTimeoutSec", 1.0);
    clearTimeoutSec = SmartDashboard.getNumber("IntakeArm/AutoClearTimeoutSec", 1.0);
    feederReverseRps = SmartDashboard.getNumber("IntakeArm/AutoClearFeederReverseRPS", -20.0);
    agitatorReverseRps = SmartDashboard.getNumber("IntakeArm/AutoClearAgitatorReverseRPS", -20.0);
    enableRecovery = SmartDashboard.getBoolean("IntakeArm/EnableAutoRecovery", true);

    state = State.ATTEMPT;
    stateTimer.restart();
    arm.setGoalDegrees(downDeg);
    SmartDashboard.putString("IntakeArm/AutoRecoveryStatus", "Attempting arm down");
  }

  @Override
  public void execute() {
    switch (state) {
      case ATTEMPT:
        arm.setGoalDegrees(downDeg);
        if (arm.atGoalRangeDeg(downDeg, tolDeg)) {
          state = State.DONE;
          SmartDashboard.putString("IntakeArm/AutoRecoveryStatus", "Arm reached target");
        } else if (stateTimer.hasElapsed(attemptTimeoutSec)) {
          if (enableRecovery) {
            state = State.CLEAR;
            stateTimer.restart();
            SmartDashboard.putString("IntakeArm/AutoRecoveryStatus", "Clearing and retrying");
          } else {
            state = State.DONE;
            SmartDashboard.putString("IntakeArm/AutoRecoveryStatus", "Timed out without recovery");
          }
        }
        break;

      case CLEAR:
        arm.setGoalDegrees(downDeg);
        feeder.setRPS(feederReverseRps);
        agitator.setRPS(agitatorReverseRps);
        if (stateTimer.hasElapsed(clearTimeoutSec)) {
          feeder.stop();
          agitator.stop();
          state = State.RETRY;
          stateTimer.restart();
          SmartDashboard.putString("IntakeArm/AutoRecoveryStatus", "Retrying arm down");
        }
        break;

      case RETRY:
        arm.setGoalDegrees(downDeg);
        if (arm.atGoalRangeDeg(downDeg, tolDeg)) {
          state = State.DONE;
          SmartDashboard.putString("IntakeArm/AutoRecoveryStatus", "Recovered successfully");
        } else if (stateTimer.hasElapsed(attemptTimeoutSec)) {
          state = State.DONE;
          SmartDashboard.putString("IntakeArm/AutoRecoveryStatus", "Retry timed out");
        }
        break;

      case DONE:
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return state == State.DONE;
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
    agitator.stop();
    arm.holdCurrentPosition();
    if (interrupted) {
      SmartDashboard.putString("IntakeArm/AutoRecoveryStatus", "Interrupted");
    }
  }
}
