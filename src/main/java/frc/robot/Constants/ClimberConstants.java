package frc.robot.Constants;

public final class ClimberConstants {

  public static final int kLeftMotorId = 37;
  public static final int kRightMotorId = 38;
  public static final String kCanBus = "rio";

  // DIO ports on the roboRIO
public static final int kLeftBottomLimitAnalogPort = 1;
  public static final int kRightBottomLimitAnalogPort = 0;
  public static final double kBottomLimitPressedThresholdVolts = 2.5;
public static final boolean kPressedWhenVoltageAboveThreshold = true;


  // Most FRC limit switches wired to DIO are active-low:
  // pressed = false from .get()
  public static final boolean kBottomLimitPressedState = false;

  public static final double kMotorRotationsPerClimberRotation = 36.0;

  // Bottom should now be zeroed by the switches
  public static final double kMinDeg = -1500.0;
  public static final double kMaxDeg = 1500.0;

  public static final double kManualUpOutput = 1;
  public static final double kManualDownOutput = .75;

  public static final double kNearZeroSlowZoneDeg = 50.0;
  public static final double kNearZeroSlowScale = 0.3;

  public static final double kP = 18.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
}