package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int cDriverControllerPort = 1;
  }
  
  /** LED related constants */
  public static class Lights {
    // PWM port on the roboRIO for AddressableLED (change to your port)
    public static final int kLedPort = 9;
    // Number of LEDs in the strip
    public static final int kLedLength = 60;
  }
}
