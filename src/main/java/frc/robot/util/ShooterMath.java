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
    if (distance <= 1.0) return 60.0;
    if (distance <= 1.5) return lerp(distance, 1.0, 60.0, 1.5, 70.0);
    if (distance <= 2.0) return lerp(distance, 1.5, 70.0, 2.0, 82.0);
    if (distance <= 2.5) return lerp(distance, 2.0, 82.0, 2.5, 95.0);
    if (distance <= 3.0) return lerp(distance, 2.5, 95.0, 3.0, 108.0);
    if (distance <= 3.5) return lerp(distance, 3.0, 108.0, 3.5, 122.0);
    if (distance <= 4.0) return lerp(distance, 3.5, 122.0, 4.0, 136.0);
    if (distance <= 4.5) return lerp(distance, 4.0, 136.0, 4.5, 150.0);
    if (distance <= 5.0) return lerp(distance, 4.5, 150.0, 5.0, 164.0);
    if (distance <= 5.5) return lerp(distance, 5.0, 164.0, 5.5, 178.0);
    if (distance <= 6.0) return lerp(distance, 5.5, 178.0, 6.0, 192.0);
    if (distance <= 6.5) return lerp(distance, 6.0, 192.0, 6.5, 205.0);
    return 215.0;
  }
}