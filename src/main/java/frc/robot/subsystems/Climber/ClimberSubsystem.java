package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {

  private final TalonFX leftMotor =
      new TalonFX(ClimberConstants.kLeftMotorId, ClimberConstants.kCanBus);

  private final TalonFX rightMotor =
      new TalonFX(ClimberConstants.kRightMotorId, ClimberConstants.kCanBus);

  private final DigitalInput leftBottomLimit =
      new DigitalInput(ClimberConstants.kLeftBottomLimitDio);

  private final DigitalInput rightBottomLimit =
      new DigitalInput(ClimberConstants.kRightBottomLimitDio);

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

    SmartDashboard.putNumber("Climber/Degrees", getDegrees());
    SmartDashboard.putNumber("Climber/LeftDegrees", getLeftDegrees());
    SmartDashboard.putNumber("Climber/RightDegrees", getRightDegrees());

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

      // If moving down, let each side stop itself at its own bottom switch
      if (movingDown) {
        if (isLeftBottomPressed()) {
          leftMotor.stopMotor();
        } else {
          leftMotor.setControl(positionRequest.withPosition(kLeftMotorSign * targetMotorRotations));
        }

        if (isRightBottomPressed()) {
          rightMotor.stopMotor();
        } else {
          rightMotor.setControl(positionRequest.withPosition(kRightMotorSign * targetMotorRotations));
        }

        return;
      }

      leftMotor.setControl(positionRequest.withPosition(kLeftMotorSign * targetMotorRotations));
      rightMotor.setControl(positionRequest.withPosition(kRightMotorSign * targetMotorRotations));
      return;
    }

    if (holdEnabled) {
      if (atMaxLimit()) {
        stopMotors();
        return;
      }

      // At the bottom, do not hold and grind into the switches
      if (isLeftBottomPressed()) {
        leftMotor.stopMotor();
      } else {
        leftMotor.setControl(holdRequest.withPosition(kLeftMotorSign * holdMotorRotations));
      }

      if (isRightBottomPressed()) {
        rightMotor.stopMotor();
      } else {
        rightMotor.setControl(holdRequest.withPosition(kRightMotorSign * holdMotorRotations));
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

  public boolean isLeftBottomPressed() {
    return leftBottomLimit.get() == ClimberConstants.kBottomLimitPressedState;
  }

  public boolean isRightBottomPressed() {
    return rightBottomLimit.get() == ClimberConstants.kBottomLimitPressedState;
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

    // Each side stops itself when its own switch is hit
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

    double clampedDegrees =
        Math.max(ClimberConstants.kMinDeg,
        Math.min(ClimberConstants.kMaxDeg, targetDegrees));

    targetMotorRotations = degreesToMotorRotations(clampedDegrees);
  }

  public void holdCurrentPosition() {
    autoPositionEnabled = false;

    // If both are at bottom, do not hold there
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
    return getDegrees() <=
        (ClimberConstants.kMinDeg + ClimberConstants.kNearZeroSlowZoneDeg);
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

    return climberRotations *
        ClimberConstants.kMotorRotationsPerClimberRotation;
  }
}