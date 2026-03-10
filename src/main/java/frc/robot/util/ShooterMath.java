package frc.robot.util;

public final class ShooterMath {
  private ShooterMath() {}

public static double tagAreaToShooterRps(double area) {
  area = Math.max(area, 0.4);

  double kA2 = 1.1176525;
  double kA1 = -14.12647126;
  double kA0 = 78.05358213;
  double offset = 0.0; // tune this first

  double rps = (kA2 * area * area) + (kA1 * area) + kA0 + offset;

  return clamp(rps, 43.0, 84.0);
}

private static double clamp(double value, double min, double max) {
  return Math.max(min, Math.min(max, value));
}
}