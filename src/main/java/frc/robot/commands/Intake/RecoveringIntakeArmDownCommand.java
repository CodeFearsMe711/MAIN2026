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
    DONE
  }

  private final IntakeArmSubsystem arm;
  private final ShooterFeederSubsytem feeder;
  private final AgitatorSubsystem agitator;
  private final Timer timer = new Timer();

  private State state = State.ATTEMPT;
  private double downDeg;
  private double tolDeg;
  private double attemptTimeoutSec;
  private double clearTimeoutSec;
  private double feederReverseRps;
  private double agitatorReverseRps;

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
    attemptTimeoutSec = SmartDashboard.getNumber("LilJohn/IntakeArmAttemptTimeoutSec", 1.5);
    clearTimeoutSec = SmartDashboard.getNumber("LilJohn/IntakeArmClearTimeoutSec", 0.5);
    feederReverseRps = SmartDashboard.getNumber("LilJohn/IntakeArmClearFeederReverseRPS", 40.0);
    agitatorReverseRps = SmartDashboard.getNumber("LilJohn/IntakeArmClearAgitatorReverseRPS", 30.0);

    state = State.ATTEMPT;
    timer.restart();
    arm.setGoalDegrees(downDeg);
    SmartDashboard.putBoolean("LilJohn/IntakeArmClearActive", false);
    SmartDashboard.putString("LilJohn/IntakeArmRecoveryStatus", "Trying intake arm down");
  }

  @Override
  public void execute() {
    switch (state) {
      case ATTEMPT:
        arm.setGoalDegrees(downDeg);
        if (arm.atGoalRangeDeg(downDeg, tolDeg)) {
          state = State.DONE;
          SmartDashboard.putString("LilJohn/IntakeArmRecoveryStatus", "Intake arm reached target");
        } else if (timer.hasElapsed(attemptTimeoutSec)) {
          state = State.CLEAR;
          timer.restart();
          SmartDashboard.putBoolean("LilJohn/IntakeArmClearActive", true);
          SmartDashboard.putString("LilJohn/IntakeArmRecoveryStatus", "Clearing intake arm");
        }
        break;

      case CLEAR:
        arm.setGoalDegrees(downDeg);
        feeder.setReverseRPS(feederReverseRps);
        agitator.setReverseRPS(agitatorReverseRps);
        if (timer.hasElapsed(clearTimeoutSec)) {
          feeder.stop();
          agitator.stop();
          SmartDashboard.putBoolean("LilJohn/IntakeArmClearActive", false);
          state = State.DONE;
          SmartDashboard.putString("LilJohn/IntakeArmRecoveryStatus", "Clear finished, continuing");
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
    SmartDashboard.putBoolean("LilJohn/IntakeArmClearActive", false);
    arm.holdCurrentPosition();
    if (interrupted) {
      SmartDashboard.putString("LilJohn/IntakeArmRecoveryStatus", "Recovery interrupted");
    }
  }
}
