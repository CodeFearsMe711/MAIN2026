package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX motor = new TalonFX(30);
  private final VelocityVoltage velocityReq = new VelocityVoltage(0);

  private boolean lowPresetHeld = false;
  private boolean fastPresetHeld = false;

  private boolean visionEnabled = false;
  private boolean visionTargetValid = false;
  private double visionTargetRPS = 0.0;

  public ShooterSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = 0.05;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kV = 0.12;

    motor.getConfigurator().apply(cfg);
  }

  public void setLowPresetHeld(boolean held) {
    lowPresetHeld = held;
  }

  public void setFastPresetHeld(boolean held) {
    fastPresetHeld = held;
  }

  public void setVisionEnabled(boolean enabled) {
    visionEnabled = enabled;

    if (!enabled) {
      visionTargetValid = false;
      visionTargetRPS = 0.0;
    }
  }

  public void updateVisionTargetRPS(double rps) {
    visionTargetRPS = rps;
    visionTargetValid = true;
  }

  public void setTargetRPS(double rps) {
    visionTargetRPS = rps;
    visionTargetValid = true;
  }

  public void setRPS(double rps) {
    setTargetRPS(rps);
  }

  public void spoolToRPS(double rps) {
    setTargetRPS(rps);
  }

  public double getActualRPS() {
    return motor.getVelocity().getValueAsDouble();
  }

  public double getActiveTargetRPS() {
    if (visionEnabled && visionTargetValid) {
      return visionTargetRPS;
    }

    if (fastPresetHeld) {
      return SmartDashboard.getNumber("Shooter/FastTargetRPS", 125.0);
    }

    if (lowPresetHeld) {
      return SmartDashboard.getNumber("Shooter/TargetRPS", 75.0);
    }

    return 0.0;
  }

  public void stop() {
    lowPresetHeld = false;
    fastPresetHeld = false;
    visionEnabled = false;
    visionTargetValid = false;
    visionTargetRPS = 0.0;
  }

  public void stopAndClearTarget() {
    stop();
  }

  @Override
  public void periodic() {
    double activeTargetRPS = getActiveTargetRPS();

    motor.setControl(velocityReq.withVelocity(activeTargetRPS));

    SmartDashboard.putBoolean("Shooter/LowPresetHeld", lowPresetHeld);
    SmartDashboard.putBoolean("Shooter/FastPresetHeld", fastPresetHeld);
    SmartDashboard.putBoolean("Shooter/VisionEnabled", visionEnabled);
    SmartDashboard.putBoolean("Shooter/VisionTargetValid", visionTargetValid);
    SmartDashboard.putNumber("Shooter/VisionTargetRPS", visionTargetRPS);
    SmartDashboard.putNumber("Shooter/ActiveTargetRPS", activeTargetRPS);
    SmartDashboard.putNumber("Shooter/MotorRPS", getActualRPS());
  }
}