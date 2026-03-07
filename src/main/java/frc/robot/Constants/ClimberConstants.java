package frc.robot.Constants;

public final class ClimberConstants {

  public static final int kLeftMotorId = 37;
  public static final int kRightMotorId = 38;
  public static final String kCanBus = "rio";

  public static final double kMotorRotationsPerClimberRotation = 100.0;

  public static final double kMinDeg = 50.0;
  public static final double kMaxDeg = 1500.0;

  public static final double kManualUpOutput = 0.6;
  public static final double kManualDownOutput = 0.3;

  public static final double kNearZeroSlowZoneDeg = 50.0;
  public static final double kNearZeroSlowScale = 0.3;

  public static final double kP = 18.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
}