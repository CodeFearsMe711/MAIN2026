package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX leftMotor =
      new TalonFX(ClimberConstants.kLeftMotorId, ClimberConstants.kCanBus);

  private final TalonFX rightMotor =
      new TalonFX(ClimberConstants.kRightMotorId, ClimberConstants.kCanBus);

  private final AnalogInput leftBottomLimit =
      new AnalogInput(ClimberConstants.kLeftBottomLimitAnalogPort);

  private final AnalogInput rightBottomLimit =
      new AnalogInput(ClimberConstants.kRightBottomLimitAnalogPort);

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
    updateBottomZeroing();
     SmartDashboard.putNumber("Climber LeftBottomVoltage", leftBottomLimit.getVoltage());
  SmartDashboard.putNumber("Climber RightBottomVoltage", rightBottomLimit.getVoltage());


    SmartDashboard.putNumber("Climber/Degrees", getDegrees());
    SmartDashboard.putNumber("ClimberLeftDegrees", getLeftDegrees());
    SmartDashboard.putNumber("ClimberRightDegrees", getRightDegrees());

    SmartDashboard.putNumber("Climber/LeftBottomVoltage", getLeftBottomVoltage());
    SmartDashboard.putNumber("Climber/RightBottomVoltage", getRightBottomVoltage());

    SmartDashboard.putBoolean("Climber/LeftBottomLimit", isLeftBottomPressed());
    SmartDashboard.putBoolean("Climber/RightBottomLimit", isRightBottomPressed());

    SmartDashboard.putBoolean("Climber/AtMinLimit", atMinLimit());
    SmartDashboard.putBoolean("Climber/AtMaxLimit", atMaxLimit());
    SmartDashboard.putBoolean("Climber/HoldEnabled", holdEnabled);
    SmartDashboard.putBoolean("Climber/AutoPosition", autoPositionEnabled);

    if (autoPositionEnabled) {
      double currentRotations = getMotorRotations();

      boolean movingUp = targetMotorRotations > currentRotations;
      boolean movingDown = targetMotorRotations < currentRotations;

      if (movingUp && atMaxLimit()) {
        stopMotors();
        return;
      }

      if (movingDown && atMinLimit()) {
        stopMotors();
        return;
      }

      if (movingDown) {
        if (isLeftBottomPressed()) {
          leftMotor.stopMotor();
          leftMotor.setPosition(0.0);
        } else {
          leftMotor.setControl(
              positionRequest.withPosition(kLeftMotorSign * targetMotorRotations));
        }

        if (isRightBottomPressed()) {
          rightMotor.stopMotor();
          rightMotor.setPosition(0.0);
        } else {
          rightMotor.setControl(
              positionRequest.withPosition(kRightMotorSign * targetMotorRotations));
        }

        return;
      }

      leftMotor.setControl(
          positionRequest.withPosition(kLeftMotorSign * targetMotorRotations));
      rightMotor.setControl(
          positionRequest.withPosition(kRightMotorSign * targetMotorRotations));
      return;
    }

    if (holdEnabled) {
      if (atMaxLimit()) {
        stopMotors();
        return;
      }

      if (isLeftBottomPressed()) {
        leftMotor.stopMotor();
        leftMotor.setPosition(0.0);
      } else {
        leftMotor.setControl(
            holdRequest.withPosition(kLeftMotorSign * holdMotorRotations));
      }

      if (isRightBottomPressed()) {
        rightMotor.stopMotor();
        rightMotor.setPosition(0.0);
      } else {
        rightMotor.setControl(
            holdRequest.withPosition(kRightMotorSign * holdMotorRotations));
      }
    }
  }

  public void zeroClimberPositionOnBoot() {
    leftMotor.setPosition(0.0);
    rightMotor.setPosition(0.0);
  }

  private void updateBottomZeroing() {
    if (isLeftBottomPressed()) {
      leftMotor.setPosition(0.0);
    }

    if (isRightBottomPressed()) {
      rightMotor.setPosition(0.0);
    }
  }

  public double getLeftBottomVoltage() {
    return leftBottomLimit.getVoltage();
  }

  public double getRightBottomVoltage() {
    return rightBottomLimit.getVoltage();
  }

  public boolean isLeftBottomPressed() {
    if (ClimberConstants.kPressedWhenVoltageAboveThreshold) {
      return getLeftBottomVoltage() >= ClimberConstants.kBottomLimitPressedThresholdVolts;
    } else {
      return getLeftBottomVoltage() <= ClimberConstants.kBottomLimitPressedThresholdVolts;
    }
  }

  public boolean isRightBottomPressed() {
    if (ClimberConstants.kPressedWhenVoltageAboveThreshold) {
      return getRightBottomVoltage() >= ClimberConstants.kBottomLimitPressedThresholdVolts;
    } else {
      return getRightBottomVoltage() <= ClimberConstants.kBottomLimitPressedThresholdVolts;
    }
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

    double output = Math.abs(ClimberConstants.kManualDownOutput);

    if (inNearZeroSlowZone()) {
      output *= ClimberConstants.kNearZeroSlowScale;
    }

    if (isLeftBottomPressed()) {
      leftMotor.stopMotor();
      leftMotor.setPosition(0.0);
    } else {
      leftMotor.setControl(manualRequest.withOutput(kLeftMotorSign * -output));
    }

    if (isRightBottomPressed()) {
      rightMotor.stopMotor();
      rightMotor.setPosition(0.0);
    } else {
      rightMotor.setControl(manualRequest.withOutput(kRightMotorSign * -output));
    }
  }

  public void setTargetDegrees(double targetDegrees) {
    holdEnabled = false;
    autoPositionEnabled = true;

    double clampedDegrees = Math.max(
        ClimberConstants.kMinDeg,
        Math.min(ClimberConstants.kMaxDeg, targetDegrees));

    targetMotorRotations = degreesToMotorRotations(clampedDegrees);
  }

  public void holdCurrentPosition() {
    autoPositionEnabled = false;

    if (atMinLimit() || atMaxLimit()) {
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
    return isLeftBottomPressed() && isRightBottomPressed();
  }

  public boolean atMaxLimit() {
    return getDegrees() >= ClimberConstants.kMaxDeg;
  }

  public boolean inNearZeroSlowZone() {
    return getDegrees()
        <= (ClimberConstants.kMinDeg + ClimberConstants.kNearZeroSlowZoneDeg);
  }

  public boolean isAtTargetDegrees(double targetDegrees, double toleranceDeg) {
    return Math.abs(getDegrees() - targetDegrees) <= toleranceDeg;
  }

  public double getDegrees() {
    return motorRotationsToDegrees(getMotorRotations());
  }

  public double getLeftDegrees() {
    return motorRotationsToDegrees(getLeftMotorRotations());
  }

  public double getRightDegrees() {
    return motorRotationsToDegrees(getRightMotorRotations());
  }

  private double getMotorRotations() {
    return (getLeftMotorRotations() + getRightMotorRotations()) / 2.0;
  }

  private double getLeftMotorRotations() {
    return kLeftMotorSign * leftMotor.getPosition().getValueAsDouble();
  }

  private double getRightMotorRotations() {
    return kRightMotorSign * rightMotor.getPosition().getValueAsDouble();
  }

  private static double motorRotationsToDegrees(double motorRotations) {
    double climberRotations =
        motorRotations / ClimberConstants.kMotorRotationsPerClimberRotation;
    return climberRotations * 360.0;
  }

  private static double degreesToMotorRotations(double degrees) {
    double climberRotations = degrees / 360.0;
    return climberRotations * ClimberConstants.kMotorRotationsPerClimberRotation;
  }
}