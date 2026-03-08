package frc.robot.util;

public final class ShooterMath {
  private ShooterMath() {}

  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  // Linear interpolation between two points
  private static double lerp(double x, double x0, double y0, double x1, double y1) {
    if (Math.abs(x1 - x0) < 1e-9) {
      return y0;
    }
    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }

  // Replace these numbers with your real tuned values
  public static double distanceMetersToShooterRps(double distanceMeters) {
    if (distanceMeters <= 1.5) {
      return 45.0;
    } else if (distanceMeters <= 2.0) {
      return lerp(distanceMeters, 1.5, 45.0, 2.0, 52.0);
    } else if (distanceMeters <= 2.5) {
      return lerp(distanceMeters, 2.0, 52.0, 2.5, 60.0);
    } else if (distanceMeters <= 3.0) {
      return lerp(distanceMeters, 2.5, 60.0, 3.0, 68.0);
    } else if (distanceMeters <= 3.5) {
      return lerp(distanceMeters, 3.0, 68.0, 3.5, 75.0);
    } else {
      return 82.0;
    }
  }

  public static double distanceMetersToShooterRpsClamped(double distanceMeters) {
    return clamp(distanceMetersToShooterRps(distanceMeters), 35.0, 90.0);
  }
}