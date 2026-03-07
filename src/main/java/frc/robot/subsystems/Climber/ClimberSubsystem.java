package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX leftMotor =
      new TalonFX(ClimberConstants.kLeftMotorId, ClimberConstants.kCanBus);
  private final TalonFX rightMotor =
      new TalonFX(ClimberConstants.kRightMotorId, ClimberConstants.kCanBus);

  private static final double kLeftMotorSign = 1.0;
  private static final double kRightMotorSign = -1.0;

  private final DutyCycleOut manualRequest = new DutyCycleOut(0.0);
  private final PositionVoltage holdRequest = new PositionVoltage(0.0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);

  private boolean holdEnabled = false;
  private boolean autoPositionEnabled = false;

  private double holdMotorRotations = 0.0;
  private double targetMotorRotations = 0.0;

  public ClimberSubsystem() {

    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = ClimberConstants.kP;
    config.Slot0.kI = ClimberConstants.kI;
    config.Slot0.kD = ClimberConstants.kD;

    leftMotor.getConfigurator().apply(config);
    rightMotor.getConfigurator().apply(config);

    zeroClimberPositionOnBoot();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Climber/Degrees", getDegrees());
    SmartDashboard.putBoolean("Climber/AtMinLimit", atMinLimit());
    SmartDashboard.putBoolean("Climber/AtMaxLimit", atMaxLimit());
    SmartDashboard.putBoolean("Climber/HoldEnabled", holdEnabled);
    SmartDashboard.putBoolean("Climber/AutoPosition", autoPositionEnabled);

    if (autoPositionEnabled) {

      double currentMotorRotations = getMotorRotations();

      boolean movingUp = targetMotorRotations > currentMotorRotations;
      boolean movingDown = targetMotorRotations < currentMotorRotations;

      if (movingUp && atMaxLimit()) {
        stopMotors();
        return;
      }

      if (movingDown && atMinLimit()) {
        stopMotors();
        return;
      }

      leftMotor.setControl(positionRequest.withPosition(kLeftMotorSign * targetMotorRotations));
      rightMotor.setControl(positionRequest.withPosition(kRightMotorSign * targetMotorRotations));
      return;
    }

    if (holdEnabled) {

      if (getDegrees() <= ClimberConstants.kMinDeg ||
          getDegrees() >= ClimberConstants.kMaxDeg) {
        stopMotors();
        return;
      }

      leftMotor.setControl(holdRequest.withPosition(kLeftMotorSign * holdMotorRotations));
      rightMotor.setControl(holdRequest.withPosition(kRightMotorSign * holdMotorRotations));
    }
  }

  public void zeroClimberPositionOnBoot() {
    leftMotor.setPosition(0.0);
    rightMotor.setPosition(0.0);
  }

  public void driveUpManual() {

    holdEnabled = false;
    autoPositionEnabled = false;

    if (atMaxLimit()) {
      stopMotors();
      return;
    }

    double output = Math.abs(ClimberConstants.kManualUpOutput);

    leftMotor.setControl(manualRequest.withOutput(kLeftMotorSign * output));
    rightMotor.setControl(manualRequest.withOutput(kRightMotorSign * output));
  }

  public void driveDownManual() {

    holdEnabled = false;
    autoPositionEnabled = false;

    if (atMinLimit()) {
      stopMotors();
      return;
    }

    double output = Math.abs(ClimberConstants.kManualDownOutput);

    if (inNearZeroSlowZone()) {
      output *= ClimberConstants.kNearZeroSlowScale;
    }

    leftMotor.setControl(manualRequest.withOutput(kLeftMotorSign * -output));
    rightMotor.setControl(manualRequest.withOutput(kRightMotorSign * -output));
  }

  public void setTargetDegrees(double targetDegrees) {

    holdEnabled = false;
    autoPositionEnabled = true;

    double clampedDegrees =
        Math.max(ClimberConstants.kMinDeg,
        Math.min(ClimberConstants.kMaxDeg, targetDegrees));

    targetMotorRotations = degreesToMotorRotations(clampedDegrees);
  }

  public void holdCurrentPosition() {

    autoPositionEnabled = false;

    if (getDegrees() <= ClimberConstants.kMinDeg ||
        getDegrees() >= ClimberConstants.kMaxDeg) {
      stopMotors();
      return;
    }

    holdMotorRotations = getMotorRotations();
    holdEnabled = true;
  }

  public void stopMotors() {
    holdEnabled = false;
    autoPositionEnabled = false;
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public boolean atMinLimit() {
    return getDegrees() <= ClimberConstants.kMinDeg;
  }

  public boolean atMaxLimit() {
    return getDegrees() >= ClimberConstants.kMaxDeg;
  }

  public boolean inNearZeroSlowZone() {
    return getDegrees() <=
        (ClimberConstants.kMinDeg + ClimberConstants.kNearZeroSlowZoneDeg);
  }

  public double getDegrees() {
    return motorRotationsToDegrees(getMotorRotations());
  }

  private double getMotorRotations() {

    double leftRot =
        kLeftMotorSign * leftMotor.getPosition().getValueAsDouble();

    double rightRot =
        kRightMotorSign * rightMotor.getPosition().getValueAsDouble();

    return (leftRot + rightRot) / 2.0;
  }

  private static double motorRotationsToDegrees(double motorRotations) {

    double climberRotations =
        motorRotations / ClimberConstants.kMotorRotationsPerClimberRotation;

    return climberRotations * 360.0;
  }

  private static double degreesToMotorRotations(double degrees) {

    double climberRotations = degrees / 360.0;

    return climberRotations *
        ClimberConstants.kMotorRotationsPerClimberRotation;
  }
}