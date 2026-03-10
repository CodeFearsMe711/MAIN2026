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

    cfg.Slot0.kP = 0.05;
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

  private double calculateTarget() {
    double now = Timer.getFPGATimestamp();

    if (visionEnabled && now <= visionHoldUntilSec) {
      return visionRPS;
    }

    if (manualFast) {
      return SmartDashboard.getNumber("Shooter/FastTargetRPS", 125.0);
    }

    if (manualLow) {
      return SmartDashboard.getNumber("Shooter/TargetRPS", 75.0);
    }

    return 0.0;
  }

  public double getMotorRPS() {
    return motor.getVelocity().getValueAsDouble();
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


 @Override
public void periodic() {
  double target = 0.0;

  if (visionEnabled) {
    target = visionRPS;
  } else if (manualFast) {
    target = SmartDashboard.getNumber("Shooter/FastTargetRPS", 125.0);
  } else if (manualLow) {
    target = SmartDashboard.getNumber("Shooter/LowPresetRPS", 75.0);
  }

  motor.setControl(velocityReq.withVelocity(target));

  SmartDashboard.putBoolean("Shooter/VisionEnabled", visionEnabled);
  SmartDashboard.putNumber("Shooter/VisionRPS", visionRPS);
  SmartDashboard.putNumber("Shooter/AppliedTargetRPS", target);
  SmartDashboard.putNumber("Shooter/MotorRPS", motor.getVelocity().getValueAsDouble());
}
}