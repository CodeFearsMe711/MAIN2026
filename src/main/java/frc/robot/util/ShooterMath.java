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
    if (distance <= 1.0) return 25.0;
    if (distance <= 1.5) return lerp(distance, 1.0, 25.0, 1.5, 30.0);
    if (distance <= 2.0) return lerp(distance, 1.5, 30.0, 2.0, 36.0);
    if (distance <= 2.5) return lerp(distance, 2.0, 36.0, 2.5, 42.0);
    if (distance <= 3.0) return lerp(distance, 2.5, 42.0, 3.0, 48.0);
    if (distance <= 3.5) return lerp(distance, 3.0, 48.0, 3.5, 54.0);
    if (distance <= 4.0) return lerp(distance, 3.5, 54.0, 4.0, 60.0);
    if (distance <= 4.5) return lerp(distance, 4.0, 60.0, 4.5, 66.0);
    if (distance <= 5.0) return lerp(distance, 4.5, 66.0, 5.0, 72.0);
    if (distance <= 5.5) return lerp(distance, 5.0, 72.0, 5.5, 78.0);
    if (distance <= 6.0) return lerp(distance, 5.5, 78.0, 6.0, 84.0);
    return 90.0;
  }
}