package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private final TalonFX motor = new TalonFX(30);
  private final VelocityVoltage velocityReq = new VelocityVoltage(0);

  private double targetRPS = 0.0;

  public ShooterSubsystem() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    cfg.Slot0.kP = 0.05;
    cfg.Slot0.kI = 0.0;
    cfg.Slot0.kD = 0.0;
    cfg.Slot0.kV = 0.12;

    motor.getConfigurator().apply(cfg);
  }

  public void setRPS(double rps) {
    setTargetRPS(rps);
  }

  public void setTargetRPS(double rps) {
    targetRPS = rps;
  }

  public void spoolToRPS(double rps) {
    setTargetRPS(rps);
  }

  public double getTargetRPS() {
    return targetRPS;
  }

  public double getActualRPS() {
    return motor.getVelocity().getValueAsDouble();
  }

  public void stop() {
    targetRPS = 0.0;
  }

  public void stopAndClearTarget() {
    stop();
  }

  @Override
  public void periodic() {
    motor.setControl(velocityReq.withVelocity(targetRPS));

    SmartDashboard.putNumber("Shooter/TargetRPS_ActualCommand", targetRPS);
    SmartDashboard.putNumber("Shooter/MotorRPS", getActualRPS());
  }
}