package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeArmConstants;

public class IntakeArmSubsystem extends SubsystemBase {
  private final TalonFX motor = new TalonFX(IntakeArmConstants.kMotorId, IntakeArmConstants.kCanBus);
  private final MotionMagicVoltage mm = new MotionMagicVoltage(0.0);

  private double goalMotorRot = 0.0;
  private boolean hasSoftZeroed = false;

  public IntakeArmSubsystem() {
    motor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = IntakeArmConstants.kP;
    cfg.Slot0.kI = IntakeArmConstants.kI;
    cfg.Slot0.kD = IntakeArmConstants.kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = armRpsToMotorRps(IntakeArmConstants.kCruiseRps_Arm);
    cfg.MotionMagic.MotionMagicAcceleration   = armRps2ToMotorRps2(IntakeArmConstants.kAccelRps2_Arm);

    motor.getConfigurator().apply(cfg);

    // Do NOT set goal here based on sensor value.
    // We will "soft zero" once on startup in periodic.
  }

  @Override
  public void periodic() {
    // Soft-zero once after startup (makes "wherever we boot" = 0)
    if (!hasSoftZeroed) {
      motor.setPosition(0);
      goalMotorRot = 0.0;
      hasSoftZeroed = true;
    }

    // Always hold the goal
    motor.setControl(mm.withPosition(goalMotorRot));
  }

  public void setGoalDegrees(double armDeg) {
    goalMotorRot = degreesToMotorRotations(armDeg);
  }

  public void setGoalZero() {
    goalMotorRot = 0.0;
  }

  public double getDegrees() {
    return motorRotationsToDegrees(getMotorRotations());
  }

  public boolean atGoal() {
    double goalDeg = motorRotationsToDegrees(goalMotorRot);
    return Math.abs(getDegrees() - goalDeg) <= IntakeArmConstants.kToleranceDeg;
  }

  /** Update Motion Magic constraints using ARM units (RPS and RPS^2). */
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
}
