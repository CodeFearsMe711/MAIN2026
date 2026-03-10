package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(30);
  private final VelocityVoltage velocityReq = new VelocityVoltage(0);

  private boolean manualLow = false;
  private boolean manualFast = false;
  private boolean visionEnabled = false;

  private double visionRPS = 0.0;
  private double visionHoldUntilSec = -1.0;

  private static final double kVisionHoldSec = 0.30;

  public ShooterSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = 0.18;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kV = 0.12;

    motor.getConfigurator().apply(cfg);
  }

  public void setLowPreset(boolean enabled) {
    manualLow = enabled;
  }

  public void setFastPreset(boolean enabled) {
    manualFast = enabled;
  }

  public void setVisionEnabled(boolean enabled) {
    visionEnabled = enabled;

    if (!enabled) {
      visionRPS = 0.0;
      visionHoldUntilSec = -1.0;
    }
  }

  public void updateVisionSpeed(double rps) {
    visionRPS = rps;
    visionHoldUntilSec = Timer.getFPGATimestamp() + kVisionHoldSec;
  }

  public void updateVisionTargetRPS(double rps) {
    updateVisionSpeed(rps);
  }

  public void clearVisionTarget() {
  }

  public void stop() {
    manualLow = false;
    manualFast = false;
    visionEnabled = false;
    visionRPS = 0.0;
    visionHoldUntilSec = -1.0;
  }

  public void stopAndClearTarget() {
    stop();
  }

  private double getTargetMotorRPS() {
    double target = 0.0;
    double now = Timer.getFPGATimestamp();

    if (visionEnabled && now <= visionHoldUntilSec) {
      target = visionRPS;
    } else if (manualFast) {
      target = SmartDashboard.getNumber("Shooter/FastTargetRPS", 70.0);
    } else if (manualLow) {
      target = SmartDashboard.getNumber("Shooter/LowPresetRPS", 35.0);
    }

    return target;
  }

  public double getMotorRPS() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    double targetMotorRPS = getTargetMotorRPS();

    motor.setControl(velocityReq.withVelocity(targetMotorRPS));

    SmartDashboard.putBoolean("Shooter/VisionEnabled", visionEnabled);
    SmartDashboard.putNumber("Shooter/VisionRPS", visionRPS);
    SmartDashboard.putNumber("Shooter/AppliedTargetRPS", targetMotorRPS);
    SmartDashboard.putNumber("Shooter/MotorRPS", getMotorRPS());
  }
}