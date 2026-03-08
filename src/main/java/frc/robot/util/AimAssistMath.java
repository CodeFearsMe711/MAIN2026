package frc.robot.util;

import java.util.List;
import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.Constants.VisionConstants;

public final class AimAssistMath {

  private AimAssistMath() {}

  public static PhotonTrackedTarget findBestAllowedTarget(
      List<PhotonTrackedTarget> targets,
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

      if (!allowed) continue;

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
      ChassisSpeeds speeds) {

    if (target == null) return Optional.empty();

    double distanceMeters =
        target.getBestCameraToTarget().getTranslation().getNorm();

    double sidewaysSpeed = speeds.vyMetersPerSecond;

    if (Math.abs(sidewaysSpeed) < 0.15) {
      sidewaysSpeed = 0;
    }

    double leadRad =
        Math.atan2(
            sidewaysSpeed
                * VisionConstants.kAimFlightTimeSec
                * VisionConstants.kAimLeadScale,
            Math.max(distanceMeters, 0.25));

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