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

  private final TalonFX motor =
      new TalonFX(IntakeArmConstants.kMotorId, IntakeArmConstants.kCanBus);

  private final MotionMagicVoltage mm = new MotionMagicVoltage(0.0);
  private final VelocityVoltage velocityReq = new VelocityVoltage(0.0);

  private double goalMotorRot = 0.0;
  private boolean hasSoftZeroed = false;

  private boolean manualControl = false;
  private double manualArmRps = 0.0;

  public IntakeArmSubsystem() {
    motor.setNeutralMode(NeutralModeValue.Brake);

    applyConfig(
        IntakeArmConstants.kP,
        IntakeArmConstants.kI,
        IntakeArmConstants.kD,
        IntakeArmConstants.kCruiseRps_Arm,
        IntakeArmConstants.kAccelRps2_Arm
    );
  }

  @Override
  public void periodic() {
    if (!hasSoftZeroed) {
      motor.setPosition(0.0);
      hasSoftZeroed = true;
    }

    SmartDashboard.putNumber("Arm Degrees", getDegrees());

    if (manualControl) {
      motor.setControl(
          velocityReq.withVelocity(
              armRpsToMotorRps(manualArmRps)
          )
      );
    } else {
      motor.setControl(
          mm.withPosition(goalMotorRot)
      );
    }
  }

  private void applyConfig(
      double kP,
      double kI,
      double kD,
      double cruiseRpsArm,
      double accelRps2Arm
  ) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = kP;
    cfg.Slot0.kI = kI;
    cfg.Slot0.kD = kD;

    cfg.MotionMagic.MotionMagicCruiseVelocity =
        armRpsToMotorRps(cruiseRpsArm);

    cfg.MotionMagic.MotionMagicAcceleration =
        armRps2ToMotorRps2(accelRps2Arm);

    motor.getConfigurator().apply(cfg);
  }

  public void enableManualArmRPS(double armRps) {
    manualArmRps = armRps;
    manualControl = true;
  }

  public void disableManualControl() {
    manualControl = false;
  }

  public void setGoalDegrees(double armDeg) {
    manualControl = false;
    goalMotorRot = degreesToMotorRotations(armDeg);
  }

  public void setGoalZero() {
    manualControl = false;
    goalMotorRot = 0.0;
  }

  public void holdCurrentPosition() {
    manualControl = false;
    goalMotorRot = getMotorRotations();
  }

  public double getDegrees() {
    return motorRotationsToDegrees(getMotorRotations());
  }

  public boolean atGoal() {
    double goalDeg = motorRotationsToDegrees(goalMotorRot);
    return Math.abs(getDegrees() - goalDeg)
        <= IntakeArmConstants.kToleranceDeg;
  }

  // ---- NEW RANGE-BASED COMPLETION ----
  public boolean atGoalRangeDeg(double goalDeg, double tolDeg) {
    return Math.abs(getDegrees() - goalDeg) <= tolDeg;
  }

  public void setMotionMagicConstraintsArm(
      double cruiseRpsArm,
      double accelRps2Arm
  ) {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    motor.getConfigurator().refresh(cfg);

    cfg.MotionMagic.MotionMagicCruiseVelocity =
        armRpsToMotorRps(cruiseRpsArm);

    cfg.MotionMagic.MotionMagicAcceleration =
        armRps2ToMotorRps2(accelRps2Arm);

    motor.getConfigurator().apply(cfg);
  }

  private double getMotorRotations() {
    return motor.getPosition().getValueAsDouble();
  }

  private static double degreesToMotorRotations(double armDeg) {
    double armRot = armDeg / 360.0;
    return armRot *
        IntakeArmConstants.kMotorRotationsPerArmRotation;
  }

  private static double motorRotationsToDegrees(double motorRot) {
    double armRot =
        motorRot /
        IntakeArmConstants.kMotorRotationsPerArmRotation;

    return armRot * 360.0;
  }

  private static double armRpsToMotorRps(double armRps) {
    return armRps *
        IntakeArmConstants.kMotorRotationsPerArmRotation;
  }

  private static double armRps2ToMotorRps2(double armRps2) {
    return armRps2 *
        IntakeArmConstants.kMotorRotationsPerArmRotation;
  }

  public boolean isSoftZeroed() {
    return hasSoftZeroed;
  }

  public void softZeroNow() {
    motor.setPosition(0.0);
    hasSoftZeroed = true;
  }
}