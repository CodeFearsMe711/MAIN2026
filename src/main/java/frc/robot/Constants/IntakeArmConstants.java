package frc.robot.Constants;

public final class IntakeArmConstants {
  private IntakeArmConstants() {}

  public static final int kMotorId = 32;
  public static final String kCanBus = "rio"; // "" also works for default

  // motorRotations = armRotations × kMotorRotationsPerArmRotation
  public static final double kMotorRotationsPerArmRotation = 45.0;

  // Editable setpoints (ARM/OUTPUT degrees)
  public static final double kPosDegA = -45.0; // DOWN (all the way down)
  public static final double kPosDegB =  0; // UP

  // Motion constraints at ARM/OUTPUT (teleop normal)
  public static final double kCruiseRps_Arm = .7;
  public static final double kAccelRps2_Arm = .5;

  // Enable drop constraints (slow, adjustable)
  public static final double kEnableCruiseRps_Arm = .35;
  public static final double kEnableAccelRps2_Arm = .25;

  // Tune these
  public static final double kP = 1;
  public static final double kI = 0.0;
  public static final double kD = 0.0;

  public static final double kToleranceDeg = 2.0;
}
