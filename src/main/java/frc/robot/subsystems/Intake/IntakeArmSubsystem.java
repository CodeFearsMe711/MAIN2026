// src/main/java/frc/robot/subsystems/Intake/IntakeArmSubsystem.java
package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeArmConstants;

public class IntakeArmSubsystem extends SubsystemBase {
  private final TalonFX motor = new TalonFX(IntakeArmConstants.kMotorId, IntakeArmConstants.kCanBus);
  private final MotionMagicVoltage mm = new MotionMagicVoltage(0.0);
  private final VelocityVoltage velocityReq = new VelocityVoltage(0);

  private double goalMotorRot = 0.0;
  private boolean hasSoftZeroed = false;

  private boolean manualControl = false;
  private double manualArmRps = 0.0;

  public IntakeArmSubsystem() {
    motor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = IntakeArmConstants.kP;
    cfg.Slot0.kI = IntakeArmConstants.kI;
    cfg.Slot0.kD = IntakeArmConstants.kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = armRpsToMotorRps(IntakeArmConstants.kCruiseRps_Arm);
    cfg.MotionMagic.MotionMagicAcceleration   = armRps2ToMotorRps2(IntakeArmConstants.kAccelRps2_Arm);

    motor.getConfigurator().apply(cfg);
  }

  @Override
  public void periodic() {
    // Soft-zero once after startup (wherever we boot = 0)
    if (!hasSoftZeroed) {
      motor.setPosition(0);
      hasSoftZeroed = true;
    }

    // Always publish angle (so it never "disappears" on Shuffleboard)
    SmartDashboard.putNumber("Arm Degrees", getDegrees());
    SmartDashboard.putBoolean("IntakeArm/ManualControl", manualControl);
    SmartDashboard.putBoolean("IntakeArm/SoftZeroed", hasSoftZeroed);

    if (manualControl) {
      motor.setControl(velocityReq.withVelocity(armRpsToMotorRps(manualArmRps)));
    } else {
      motor.setControl(mm.withPosition(goalMotorRot));
    }
  }

  public void enableManualArmRPS(double armRps) {
    manualArmRps = armRps;
    manualControl = true;
  }

  public void disableManualControl() {
    manualControl = false;
  }

  // ✅ Key reliability fix: any position goal forces Motion Magic mode
  public void setGoalDegrees(double armDeg) {
    manualControl = false;
    goalMotorRot = degreesToMotorRotations(armDeg);
  }

  public void setGoalZero() {
    manualControl = false;
    goalMotorRot = 0.0;
  }

  // ✅ Used by Robot.disabledInit() so re-enable never feels "dead"
  public void holdCurrentPosition() {
    manualControl = false;
    goalMotorRot = getMotorRotations();
  }

  public double getDegrees() {
    return motorRotationsToDegrees(getMotorRotations());
  }

  public boolean atGoal() {
    double goalDeg = motorRotationsToDegrees(goalMotorRot);
    return Math.abs(getDegrees() - goalDeg) <= IntakeArmConstants.kToleranceDeg;
  }

  public void setMotionMagicConstraintsArm(double cruiseRpsArm, double accelRps2Arm) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    motor.getConfigurator().refresh(cfg);

    cfg.MotionMagic.MotionMagicCruiseVelocity = armRpsToMotorRps(cruiseRpsArm);
    cfg.MotionMagic.MotionMagicAcceleration   = armRps2ToMotorRps2(accelRps2Arm);

    motor.getConfigurator().apply(cfg);
  }

  private double getMotorRotations() {
    return motor.getPosition().getValueAsDouble();
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

  private static double armRps2ToMotorRps2(double armRps2) {
    return armRps2 * IntakeArmConstants.kMotorRotationsPerArmRotation;
  }

  public boolean isSoftZeroed() {
    return hasSoftZeroed;
  }

  public void softZeroNow() {
    motor.setPosition(0);
    hasSoftZeroed = true;
  }
}