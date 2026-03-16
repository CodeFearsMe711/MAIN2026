package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(30);
  private final VoltageOut voltageOut = new VoltageOut(0);

  private boolean manualEnabled = false;
  private boolean visionEnabled = false;

  private double manualTargetRPS = 0.0;
  private double visionTargetRPS = 0.0;

  private double appliedVolts = 0.0;

  private static final double MAX_RPS = 90.0;
  private static final double MAX_VOLTS = 12.0;

  public ShooterSubsystem() {
    SmartDashboard.putString("ZZZ_SHOOTER_CODE_VERSION", "NEW_SHOOTER_BUILD");
  }

  public void setManualRPS(double rps) {
    manualEnabled = true;
    manualTargetRPS = rps;
  }

  public void clearManualRPS() {
    manualEnabled = false;
    manualTargetRPS = 0.0;
  }

  public void setVisionEnabled(boolean enabled) {
    visionEnabled = enabled;
    if (!enabled) {
      visionTargetRPS = 0.0;
    }
  }

  public void updateVisionSpeed(double rps) {
    visionTargetRPS = rps;
  }

  private double getRequestedRPS() {

    if (visionEnabled) {
      return visionTargetRPS;
    }

    if (manualEnabled) {
      return manualTargetRPS;
    }

    return 0.0;
  }

  private double rpsToVoltage(double rps) {
    return (rps / MAX_RPS) * MAX_VOLTS;
  }

  public double getMotorRPS() {
    return motor.getVelocity().getValueAsDouble();
  }

  @Override
public void periodic() {

  // PROOF periodic is running
  

  double requestedRPS = getRequestedRPS();

  double volts = (requestedRPS / MAX_RPS) * MAX_VOLTS;

  motor.setControl(voltageOut.withOutput(volts));

}
}