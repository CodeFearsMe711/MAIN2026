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

  public IntakeArmSubsystem() {
    motor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = IntakeArmConstants.kP;
    cfg.Slot0.kI = IntakeArmConstants.kI;
    cfg.Slot0.kD = IntakeArmConstants.kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity = armRpsToMotorRps(IntakeArmConstants.kCruiseRps_Arm);
    cfg.MotionMagic.MotionMagicAcceleration   = armRps2ToMotorRps2(IntakeArmConstants.kAccelRps2_Arm);

    motor.getConfigurator().apply(cfg);

    goalMotorRot = getMotorRotations(); // hold wherever we start
  }

  @Override
  public void periodic() {
    // Always hold the last goal to prevent sagging
    motor.setControl(mm.withPosition(goalMotorRot));
  }

  public void setGoalDegrees(double armDeg) {
    goalMotorRot = degreesToMotorRotations(armDeg);
  }

  public double getDegrees() {
    return motorRotationsToDegrees(getMotorRotations());
  }

  public boolean atGoal() {
    double goalDeg = motorRotationsToDegrees(goalMotorRot);
    return Math.abs(getDegrees() - goalDeg) <= IntakeArmConstants.kToleranceDeg;
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
