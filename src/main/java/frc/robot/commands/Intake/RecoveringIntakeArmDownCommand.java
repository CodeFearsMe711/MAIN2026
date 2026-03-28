package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
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
  private double feederReverseVolts;
  private double agitatorReverseVolts;

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
    attemptTimeoutSec = SmartDashboard.getNumber("LilJohnIntakeArmAttemptTimeoutSec", 1.5);
    clearTimeoutSec = SmartDashboard.getNumber("LilJohnIntakeArmClearTimeoutSec", 0.5);
    feederReverseVolts = SmartDashboard.getNumber("LilJohnIntakeArmClearFeederReverseVolts", 10.0);
    agitatorReverseVolts = SmartDashboard.getNumber("LilJohnIntakeArmClearAgitatorReverseVolts", 10.0);

    state = State.ATTEMPT;
    timer.restart();
    arm.setGoalDegrees(downDeg);
    SmartDashboard.putBoolean("LilJohnIntakeArmClearActive", false);
    SmartDashboard.putBoolean("LilJohnIntakeArmClearTriggered", false);
    SmartDashboard.putString("LilJohnIntakeArmRecoveryStatus", "Trying intake arm down");
    DriverStation.reportWarning("LilJohn intake-arm recovery command initialized", false);
  }

  @Override
  public void execute() {
    switch (state) {
      case ATTEMPT:
        arm.setGoalDegrees(downDeg);
        if (arm.atGoalRangeDeg(downDeg, tolDeg)) {
          state = State.DONE;
          SmartDashboard.putString("LilJohnIntakeArmRecoveryStatus", "Intake arm reached target");
        } else if (timer.hasElapsed(attemptTimeoutSec)) {
          state = State.CLEAR;
          timer.restart();
          SmartDashboard.putBoolean("LilJohnIntakeArmClearActive", true);
          SmartDashboard.putBoolean("LilJohnIntakeArmClearTriggered", true);
          SmartDashboard.putNumber(
              "LilJohnIntakeArmClearCount",
              SmartDashboard.getNumber("LilJohnIntakeArmClearCount", 0.0) + 1.0);
          SmartDashboard.putString("LilJohnIntakeArmRecoveryStatus", "Clearing intake arm");
          DriverStation.reportWarning("LilJohn intake-arm clear phase active", false);
        }
        break;

      case CLEAR:
        arm.setGoalDegrees(downDeg);
        feeder.setReverseVoltage(feederReverseVolts);
        agitator.setReverseVoltage(agitatorReverseVolts);
        if (timer.hasElapsed(clearTimeoutSec)) {
          feeder.stop();
          agitator.stop();
          SmartDashboard.putBoolean("LilJohnIntakeArmClearActive", false);
          state = State.DONE;
          SmartDashboard.putString("LilJohnIntakeArmRecoveryStatus", "Clear finished, continuing");
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
    SmartDashboard.putBoolean("LilJohnIntakeArmClearActive", false);
    arm.holdCurrentPosition();
    if (interrupted) {
      SmartDashboard.putString("LilJohnIntakeArmRecoveryStatus", "Recovery interrupted");
    }
  }
}
