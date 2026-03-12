package frc.robot.util;

public final class ShooterMath {
  private ShooterMath() {}

public static double tagAreaToShooterRps(double area) {
  area = Math.max(area, 0.4);

  double kA2 = 1.1176525;
  double kA1 = -14.12647126;
  double kA0 = 79.05358213;
  double offset = 0.0;

  double farBoost = 0.0;
  if (area < 0.8) {
    farBoost = (0.8 - area) * 18.0;
  }

  double rps = (kA2 * area * area) + (kA1 * area) + kA0 + offset + farBoost;

  return clamp(rps, 60.0, 150.0);
}

private static double clamp(double value, double min, double max) {
  return Math.max(min, Math.min(max, value));
}
}