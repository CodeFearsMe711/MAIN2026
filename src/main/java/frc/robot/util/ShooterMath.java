package frc.robot.util;

public final class ShooterMath {
  private ShooterMath() {}

  private static double lerp(double x, double x0, double y0, double x1, double y1) {
    if (Math.abs(x1 - x0) < 1e-9) {
      return y0;
    }
    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }

  public static double distanceMetersToShooterRps(double distance) {
    if (distance <= 1.5) return 75.0;
    if (distance <= 2.0) return lerp(distance, 1.5, 75.0, 2.0, 95.0);
    if (distance <= 2.5) return lerp(distance, 2.0, 95.0, 2.5, 115.0);
    if (distance <= 3.0) return lerp(distance, 2.5, 115.0, 3.0, 135.0);
    if (distance <= 3.5) return lerp(distance, 3.0, 135.0, 3.5, 155.0);
    if (distance <= 4.0) return lerp(distance, 3.5, 155.0, 4.0, 175.0);
    return 190.0;
  }
}