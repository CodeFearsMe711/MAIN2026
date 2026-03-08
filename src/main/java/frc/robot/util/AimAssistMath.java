package frc.robot.util;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Constants.VisionConstants;

public final class AimAssistMath {
  private AimAssistMath() {}

  public static PhotonTrackedTarget findBestAllowedTarget(
      java.util.List<PhotonTrackedTarget> targets,
      int... allowedIds) {
    PhotonTrackedTarget best = null;
    double bestAbsYaw = Double.POSITIVE_INFINITY;

    for (PhotonTrackedTarget t : targets) {
      int id = t.getFiducialId();

      boolean allowed = false;
      for (int a : allowedIds) {
        if (id == a) {
          allowed = true;
          break;
        }
      }
      if (!allowed) {
        continue;
      }

      double absYaw = Math.abs(t.getYaw());
      if (absYaw < bestAbsYaw) {
        bestAbsYaw = absYaw;
        best = t;
      }
    }

    return best;
  }

  public static Optional<Double> getCorrectedYawRad(
      PhotonTrackedTarget target,
      ChassisSpeeds robotRelativeSpeeds) {
    if (target == null) {
      return Optional.empty();
    }

    double x = target.getBestCameraToTarget().getTranslation().getX();
    double y = target.getBestCameraToTarget().getTranslation().getY();
    double dist = Math.hypot(x, y);

    if (dist < 1e-6) {
      double correctedYawDeg = target.getYaw() + VisionConstants.kAimYawOffsetDeg;
      return Optional.of(Units.degreesToRadians(correctedYawDeg));
    }

    double ux = x / dist;
    double uy = y / dist;

    // Perpendicular to line of sight in robot/camera plane
    double nx = -uy;
    double ny = ux;

    double vx = robotRelativeSpeeds.vxMetersPerSecond;
    double vy = robotRelativeSpeeds.vyMetersPerSecond;

    // Positive means robot motion is carrying the shot across the line of sight
    double lateralSpeed = vx * nx + vy * ny;

    double leadRad =
        Math.atan2(
            -lateralSpeed
                * VisionConstants.kAimFlightTimeSec
                * VisionConstants.kAimLeadScale,
            Math.max(dist, 0.25));

    double leadDeg = Units.radiansToDegrees(leadRad);
    leadDeg =
        Math.max(
            -VisionConstants.kAimMaxLeadDeg,
            Math.min(VisionConstants.kAimMaxLeadDeg, leadDeg));

    double correctedYawDeg =
        target.getYaw()
            + VisionConstants.kAimYawOffsetDeg
            + leadDeg;

    return Optional.of(Units.degreesToRadians(correctedYawDeg));
  }

  public static double getDistanceMeters(PhotonTrackedTarget target) {
    return target.getBestCameraToTarget().getTranslation().getNorm();
  }
}