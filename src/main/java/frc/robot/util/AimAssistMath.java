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
    return findBestAllowedTarget(targets, -1, allowedIds);
  }

  public static PhotonTrackedTarget findBestAllowedTarget(
      List<PhotonTrackedTarget> targets,
      int preferredId,
      int... allowedIds) {

    if (preferredId >= 0) {
      for (PhotonTrackedTarget t : targets) {
        if (t.getFiducialId() == preferredId && isAllowed(preferredId, allowedIds) && isUsable(t)) {
          return t;
        }
      }
    }

    PhotonTrackedTarget best = null;
    double bestScore = Double.POSITIVE_INFINITY;

    for (PhotonTrackedTarget t : targets) {
      int id = t.getFiducialId();
      if (!isAllowed(id, allowedIds) || !isUsable(t)) {
        continue;
      }

      double ambiguity = t.getPoseAmbiguity();
      double ambiguityPenalty = ambiguity >= 0.0 ? ambiguity * 6.0 : 0.0;
      double score = Math.abs(t.getYaw()) - (2.0 * t.getArea()) + ambiguityPenalty;

      if (score < bestScore) {
        bestScore = score;
        best = t;
      }
    }

    return best;
  }

  private static boolean isAllowed(int id, int... allowedIds) {
    for (int a : allowedIds) {
      if (id == a) {
        return true;
      }
    }
    return false;
  }

  private static boolean isUsable(PhotonTrackedTarget target) {
    double ambiguity = target.getPoseAmbiguity();
    return ambiguity < 0.0 || ambiguity <= VisionConstants.kMaxPoseAmbiguity;
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
