package frc.robot.Constants;

public final class ClimberConstants {
  private ClimberConstants() {}

  // CAN IDs for the two Kraken X60s (assign defaults; update to match your robot wiring)
  public static final int kLeftMotorId = 37;
  public static final int kRightMotorId = 38;
  public static final String kCanBus = "rio";

  // motorRotations = climberRotations × kMotorRotationsPerClimberRotation
  // Adjust this to match your gearing
  public static final double kMotorRotationsPerClimberRotation = 135;

  // Default software limits (degrees)
  public static final double kDefaultMinDeg = 0.0;
  public static final double kDefaultMaxDeg = 90.0;

  // Motion constraints and PID (tune on robot)
  public static final double kCruiseRps = 4;
  public static final double kAccelRps2 = 2;

  // MotionMagic / closed-loop PID tuning
  public static final double kP = 1.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kToleranceDeg = 2.0;
}
