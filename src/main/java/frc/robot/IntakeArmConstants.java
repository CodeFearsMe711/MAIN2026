package frc.robot;

public final class IntakeArmConstants {
  private IntakeArmConstants() {}

  public static final int kMotorId = 15;
  public static final String kCanBus = "rio"; // "" also works for default

  // motorRotations = armRotations × kMotorRotationsPerArmRotation
  // 1:1 for now. If you later have 25:1 motor:arm, set to 25.0
  public static final double kMotorRotationsPerArmRotation = 1.0;

  // Editable setpoints (ARM/OUTPUT degrees)
  public static final double kPosDegA = 0.0;
  public static final double kPosDegB = 90.0;

  // Motion constraints at ARM/OUTPUT
  public static final double kCruiseRps_Arm = 0.5;
  public static final double kAccelRps2_Arm = 0.5;

  // Tune these
  public static final double kP = 50.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kToleranceDeg = 2.0;
}
