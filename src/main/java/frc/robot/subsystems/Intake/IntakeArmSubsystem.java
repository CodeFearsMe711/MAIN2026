package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmConstants;

public class IntakeArmSubsystem extends SubsystemBase {
  private enum ControlMode {
    POSITION,
    MANUAL_VELOCITY
  }

  private static final double kLoopDtSec = 0.02;

  private final TalonFX motor =
      new TalonFX(IntakeArmConstants.kMotorId, IntakeArmConstants.kCanBus);

  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  private final StatusSignal<edu.wpi.first.units.measure.Angle> positionSignal = motor.getPosition();
  private final StatusSignal<edu.wpi.first.units.measure.AngularVelocity> velocitySignal = motor.getVelocity();

  private ControlMode controlMode = ControlMode.POSITION;

  private boolean bootZeroDone = false;

  private double desiredGoalDeg = IntakeArmConstants.kPosDegB;
  private double manualArmRps = 0.0;

  private double maxProfileVelDegPerSec = armRpsToDegPerSec(IntakeArmConstants.kCruiseRps_Arm);
  private double maxProfileAccelDegPerSec2 = armRpsToDegPerSec(IntakeArmConstants.kAccelRps2_Arm);

  private TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(maxProfileVelDegPerSec, maxProfileAccelDegPerSec2);

  private TrapezoidProfile.State profiledSetpointDeg =
      new TrapezoidProfile.State(IntakeArmConstants.kPosDegB, 0.0);

  private double lastPositionCommandMotorRot = Double.NaN;
  private double lastVelocityCommandMotorRps = Double.NaN;
  private double lastPublishedDegrees = Double.NaN;

  public IntakeArmSubsystem() {
    motor.setNeutralMode(NeutralModeValue.Brake);
    applyBaseConfig();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, positionSignal, velocitySignal);
    motor.optimizeBusUtilization();

    BaseStatusSignal.refreshAll(positionSignal, velocitySignal);

    double startDeg = clampDegrees(getDegrees());
    desiredGoalDeg = startDeg;
    profiledSetpointDeg = new TrapezoidProfile.State(startDeg, 0.0);

    SmartDashboard.putBoolean("IntakeArm/BootZeroDone", false);
    SmartDashboard.putNumber("Arm Degrees", startDeg);
  }

  @Override
  public void periodic() {
    BaseStatusSignal.refreshAll(positionSignal, velocitySignal);

    double currentDeg = getDegrees();

    if (Double.isNaN(lastPublishedDegrees) || Math.abs(currentDeg - lastPublishedDegrees) >= 0.1) {
      SmartDashboard.putNumber("Arm Degrees", currentDeg);
      lastPublishedDegrees = currentDeg;
    }

    if (controlMode == ControlMode.MANUAL_VELOCITY) {
      applyManualVelocity();
    } else {
      applyProfiledPosition();
    }
  }

  private void applyBaseConfig() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = IntakeArmConstants.kP;
    cfg.Slot0.kI = IntakeArmConstants.kI;
    cfg.Slot0.kD = IntakeArmConstants.kD;

    motor.getConfigurator().apply(cfg);
  }

  private void applyProfiledPosition() {
    double currentDeg = clampDegrees(getDegrees());
    double currentDegPerSec = motorRpsToDegPerSec(velocitySignal.getValueAsDouble());

    TrapezoidProfile profile = new TrapezoidProfile(profileConstraints);

    TrapezoidProfile.State measuredState =
        new TrapezoidProfile.State(currentDeg, currentDegPerSec);

    TrapezoidProfile.State goalState =
        new TrapezoidProfile.State(clampDegrees(desiredGoalDeg), 0.0);

    double posErrorToMeasured = Math.abs(profiledSetpointDeg.position - currentDeg);

    if (Double.isNaN(profiledSetpointDeg.position) || posErrorToMeasured > 8.0) {
      profiledSetpointDeg = measuredState;
    }

    profiledSetpointDeg = profile.calculate(kLoopDtSec, profiledSetpointDeg, goalState);

    double targetMotorRot = degreesToMotorRotations(profiledSetpointDeg.position);

    if (Double.isNaN(lastPositionCommandMotorRot)
        || Math.abs(targetMotorRot - lastPositionCommandMotorRot) > 1e-4) {
      motor.setControl(positionRequest.withPosition(targetMotorRot));
      lastPositionCommandMotorRot = targetMotorRot;
    }
  }

  private void applyManualVelocity() {
    double currentDeg = getDegrees();

    double commandedArmRps = manualArmRps;

    if (currentDeg <= getMinDeg() && commandedArmRps < 0.0) {
      commandedArmRps = 0.0;
    }
    if (currentDeg >= getMaxDeg() && commandedArmRps > 0.0) {
      commandedArmRps = 0.0;
    }

    double targetMotorRps = armRpsToMotorRps(commandedArmRps);

    if (Double.isNaN(lastVelocityCommandMotorRps)
        || Math.abs(targetMotorRps - lastVelocityCommandMotorRps) > 1e-4) {
      motor.setControl(velocityRequest.withVelocity(targetMotorRps));
      lastVelocityCommandMotorRps = targetMotorRps;
    }
  }

  public void zeroArmPositionOnBoot() {
    if (bootZeroDone) {
      return;
    }

    motor.setPosition(degreesToMotorRotations(IntakeArmConstants.kPosDegB));
    bootZeroDone = true;

    BaseStatusSignal.refreshAll(positionSignal, velocitySignal);

    double currentDeg = clampDegrees(getDegrees());
    desiredGoalDeg = currentDeg;
    profiledSetpointDeg = new TrapezoidProfile.State(currentDeg, 0.0);

    invalidateCachedCommands();

    SmartDashboard.putBoolean("IntakeArm/BootZeroDone", true);
  }

  public boolean atGoalRangeDeg(double goalDeg, double tolDeg) {
    return Math.abs(getDegrees() - clampDegrees(goalDeg)) <= Math.abs(tolDeg);
  }

  public void enableManualArmRPS(double armRps) {
    manualArmRps = armRps;
    controlMode = ControlMode.MANUAL_VELOCITY;
    lastVelocityCommandMotorRps = Double.NaN;
  }

  public void disableManualControl() {
    controlMode = ControlMode.POSITION;

    double currentDeg = clampDegrees(getDegrees());
    profiledSetpointDeg =
        new TrapezoidProfile.State(
            currentDeg,
            motorRpsToDegPerSec(velocitySignal.getValueAsDouble()));

    lastPositionCommandMotorRot = Double.NaN;
  }

  public void setGoalDegrees(double armDeg) {
    desiredGoalDeg = clampDegrees(armDeg);
    controlMode = ControlMode.POSITION;
  }

  public void holdCurrentPosition() {
    double currentDeg = clampDegrees(getDegrees());
    desiredGoalDeg = currentDeg;
    controlMode = ControlMode.POSITION;
    profiledSetpointDeg = new TrapezoidProfile.State(currentDeg, 0.0);
    lastPositionCommandMotorRot = Double.NaN;
  }

  public double getDegrees() {
    return motorRotationsToDegrees(positionSignal.getValueAsDouble());
  }

  public void setMotionMagicConstraintsArm(double cruiseRpsArm, double accelRps2Arm) {
    double safeCruise = Math.max(0.001, cruiseRpsArm);
    double safeAccel = Math.max(0.001, accelRps2Arm);

    maxProfileVelDegPerSec = armRpsToDegPerSec(safeCruise);
    maxProfileAccelDegPerSec2 = armRpsToDegPerSec2(safeAccel);

    profileConstraints =
        new TrapezoidProfile.Constraints(maxProfileVelDegPerSec, maxProfileAccelDegPerSec2);
  }

  private void invalidateCachedCommands() {
    lastPositionCommandMotorRot = Double.NaN;
    lastVelocityCommandMotorRps = Double.NaN;
  }

  private double getMinDeg() {
    return Math.min(IntakeArmConstants.kPosDegA, IntakeArmConstants.kPosDegB);
  }

  private double getMaxDeg() {
    return Math.max(IntakeArmConstants.kPosDegA, IntakeArmConstants.kPosDegB);
  }

  private double clampDegrees(double armDeg) {
    return MathUtil.clamp(armDeg, getMinDeg(), getMaxDeg());
  }

  private static double degreesToMotorRotations(double armDeg) {
    double armRot = armDeg / 360.0;
    return armRot * IntakeArmConstants.kMotorRotationsPerArmRotation;
  }

  private static double motorRotationsToDegrees(double motorRot) {
    double armRot = motorRot / IntakeArmConstants.kMotorRotationsPerArmRotation;
    return armRot * 360.0;
  }

  private static double armRpsToMotorRps(double armRps) {
    return armRps * IntakeArmConstants.kMotorRotationsPerArmRotation;
  }

  private static double armRpsToDegPerSec(double armRps) {
    return armRps * 360.0;
  }

  private static double armRpsToDegPerSec2(double armRps2) {
    return armRps2 * 360.0;
  }

  private static double motorRpsToDegPerSec(double motorRps) {
    double armRps = motorRps / IntakeArmConstants.kMotorRotationsPerArmRotation;
    return armRps * 360.0;
  }
}