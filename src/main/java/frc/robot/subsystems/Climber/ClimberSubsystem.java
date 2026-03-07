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
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final PositionVoltage holdRequest = new PositionVoltage(0.0);

  private boolean manualMode = false;
  private boolean autoPositionMode = false;

  private double manualOutput = 0.0;
  private double holdMotorRotations = 0.0;
  private double targetMotorRotations = 0.0;

  private boolean bootZeroDone = false;
  private boolean holdEnabled = false;

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
    stopMotors();

    SmartDashboard.putNumber("Climber/Degrees", 0.0);
    SmartDashboard.putNumber("Climber/HoldDeg", 0.0);
    SmartDashboard.putNumber("Climber/TargetDeg", 0.0);
    SmartDashboard.putBoolean("Climber/AtMinLimit", false);
    SmartDashboard.putBoolean("Climber/AtMaxLimit", false);
    SmartDashboard.putBoolean("Climber/InMinStopZone", false);
    SmartDashboard.putBoolean("Climber/InMaxStopZone", false);
    SmartDashboard.putBoolean("Climber/InNearZeroSlowZone", false);
    SmartDashboard.putBoolean("Climber/HoldEnabled", false);
    SmartDashboard.putBoolean("Climber/AutoPositionMode", false);
    SmartDashboard.putNumber("Climber/AppliedManualOutput", 0.0);
    SmartDashboard.putNumber("Climber/Left Stator Current", 0.0);
    SmartDashboard.putNumber("Climber/Right Stator Current", 0.0);
  }

  @Override
  public void periodic() {
    double currentDeg = getDegrees();

    SmartDashboard.putNumber("Climber/Degrees", currentDeg);
    SmartDashboard.putNumber("Climber/HoldDeg", motorRotationsToDegrees(holdMotorRotations));
    SmartDashboard.putNumber("Climber/TargetDeg", motorRotationsToDegrees(targetMotorRotations));
    SmartDashboard.putBoolean("Climber/AtMinLimit", atMinLimit());
    SmartDashboard.putBoolean("Climber/AtMaxLimit", atMaxLimit());
    SmartDashboard.putBoolean("Climber/InMinStopZone", inMinStopZone());
    SmartDashboard.putBoolean("Climber/InMaxStopZone", inMaxStopZone());
    SmartDashboard.putBoolean("Climber/InNearZeroSlowZone", inNearZeroSlowZone());
    SmartDashboard.putBoolean("Climber/HoldEnabled", holdEnabled);
    SmartDashboard.putBoolean("Climber/AutoPositionMode", autoPositionMode);
    SmartDashboard.putNumber("Climber/AppliedManualOutput", manualOutput);
    SmartDashboard.putNumber("Climber/Left Stator Current", leftMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Climber/Right Stator Current", rightMotor.getStatorCurrent().getValueAsDouble());

    if (manualMode) {
      double commanded = manualOutput;

      if (commanded > 0.0 && inMaxStopZone()) {
        stopMotors();
        return;
      }

      if (commanded < 0.0 && inMinStopZone()) {
        stopMotors();
        return;
      }

      leftMotor.setControl(manualRequest.withOutput(kLeftMotorSign * commanded));
      rightMotor.setControl(manualRequest.withOutput(kRightMotorSign * commanded));
      return;
    }

    if (autoPositionMode) {
      double currentMotorRotations = getMotorRotations();

      if (targetMotorRotations > currentMotorRotations && inMaxStopZone()) {
        stopMotors();
        return;
      }

      if (targetMotorRotations < currentMotorRotations && inMinStopZone()) {
        stopMotors();
        return;
      }

      leftMotor.setControl(positionRequest.withPosition(kLeftMotorSign * targetMotorRotations));
      rightMotor.setControl(positionRequest.withPosition(kRightMotorSign * targetMotorRotations));
      return;
    }

    if (!holdEnabled) {
      leftMotor.stopMotor();
      rightMotor.stopMotor();
      return;
    }

    if (inMinStopZone() || inMaxStopZone()) {
      stopMotors();
      return;
    }

    leftMotor.setControl(holdRequest.withPosition(kLeftMotorSign * holdMotorRotations));
    rightMotor.setControl(holdRequest.withPosition(kRightMotorSign * holdMotorRotations));
  }

  public void zeroClimberPositionOnBoot() {
    if (bootZeroDone) return;

    leftMotor.setPosition(0.0);
    rightMotor.setPosition(0.0);
    bootZeroDone = true;
  }

  public void driveUp() {
    if (inMaxStopZone()) {
      stopMotors();
      return;
    }

    autoPositionMode = false;
    holdEnabled = false;
    manualMode = true;
    manualOutput = Math.abs(ClimberConstants.kManualUpOutput);
  }

  public void driveDown() {
    if (inMinStopZone()) {
      stopMotors();
      return;
    }

    autoPositionMode = false;
    holdEnabled = false;
    manualMode = true;

    double downOutput = Math.abs(ClimberConstants.kManualDownOutput);

    if (inNearZeroSlowZone()) {
      downOutput *= ClimberConstants.kNearZeroSlowScale;
    }

    manualOutput = -downOutput;
  }

  public void setTargetDegrees(double targetDegrees) {
    double clampedDegrees = Math.max(ClimberConstants.kMinDeg,
        Math.min(ClimberConstants.kMaxDeg, targetDegrees));

    manualMode = false;
    manualOutput = 0.0;
    holdEnabled = false;
    autoPositionMode = true;
    targetMotorRotations = degreesToMotorRotations(clampedDegrees);
  }

  public void holdCurrentPosition() {
    manualMode = false;
    autoPositionMode = false;
    manualOutput = 0.0;

    if (inMinStopZone() || inMaxStopZone()) {
      stopMotors();
      return;
    }

    holdMotorRotations = getMotorRotations();
    holdEnabled = true;
  }

  public void stopMotors() {
    manualMode = false;
    autoPositionMode = false;
    manualOutput = 0.0;
    holdEnabled = false;
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public boolean atMinLimit() {
    return getDegrees() <= ClimberConstants.kMinDeg;
  }

  public boolean atMaxLimit() {
    return getDegrees() >= ClimberConstants.kMaxDeg;
  }

  public boolean inMinStopZone() {
    return getDegrees() <= (ClimberConstants.kMinDeg + ClimberConstants.kLimitStopZoneDeg);
  }

  public boolean inMaxStopZone() {
    return getDegrees() >= (ClimberConstants.kMaxDeg - ClimberConstants.kLimitStopZoneDeg);
  }

  public boolean inNearZeroSlowZone() {
    return getDegrees() <= (ClimberConstants.kMinDeg + ClimberConstants.kNearZeroSlowZoneDeg);
  }

  public boolean isAtTargetDegrees(double targetDegrees, double toleranceDeg) {
    return Math.abs(getDegrees() - targetDegrees) <= toleranceDeg;
  }

  public double getDegrees() {
    return motorRotationsToDegrees(getMotorRotations());
  }

  private double getMotorRotations() {
    double leftRot = kLeftMotorSign * leftMotor.getPosition().getValueAsDouble();
    double rightRot = kRightMotorSign * rightMotor.getPosition().getValueAsDouble();
    return (leftRot + rightRot) / 2.0;
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