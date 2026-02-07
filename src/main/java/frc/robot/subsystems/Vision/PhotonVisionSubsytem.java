// src/main/java/frc/robot/subsystems/Vision/PhotonVisionSubsytem.java
package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.Constants.VisionConstants;

public class PhotonVisionSubsytem extends SubsystemBase {

  private final PhotonCamera camera;
  private final AprilTagFieldLayout fieldLayout;
  private final PhotonPoseEstimator poseEstimator;

  public PhotonVisionSubsytem() {
    camera = new PhotonCamera(VisionConstants.kCameraName);
    fieldLayout = loadFieldLayoutRobust();

    // PhotonLib 2026 constructor: (fieldLayout, robotToCamera)
    poseEstimator = new PhotonPoseEstimator(fieldLayout, VisionConstants.kRobotToCamera);
  }

  private AprilTagFieldLayout loadFieldLayoutRobust() {
    String[] candidates = new String[] {
        "k2026RebuiltWelded",
        "k2026RebuiltAndymark",
        "k2025ReefscapeWelded",
        "k2025ReefscapeAndyMark",
        "k2024Crescendo",
        "k2023ChargedUp",
        "kDefaultField"
    };

    for (String name : candidates) {
      try {
        AprilTagFields f = AprilTagFields.valueOf(name);
        return AprilTagFieldLayout.loadField(f);
      } catch (Exception ignored) {}
    }

    return AprilTagFieldLayout.loadField(AprilTagFields.values()[0]);
  }

  /** Returns a filtered vision pose estimate (field-relative). */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d referencePose2d) {

    DriverStation.getAlliance().ifPresent(alliance -> {
      if (alliance == DriverStation.Alliance.Red) {
        fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
      } else {
        fieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
      }
    });

    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) return Optional.empty();

    // Prefer coprocessor multitag (enable in PhotonVision UI)
    Optional<EstimatedRobotPose> est = poseEstimator.estimateCoprocMultiTagPose(result);

    // Stable fallback: closest to current reference pose
    if (est.isEmpty()) {
      est = poseEstimator.estimateClosestToReferencePose(result, new Pose3d(referencePose2d));
    }

    // Last fallback: lowest ambiguity single-tag
    if (est.isEmpty()) {
      est = poseEstimator.estimateLowestAmbiguityPose(result);
    }

    if (est.isEmpty()) return Optional.empty();

    EstimatedRobotPose out = est.get();
    if (!isGoodEstimate(out)) return Optional.empty();

    return Optional.of(out);
  }

  /** For teleop hold-to-aim: returns yaw error (rad) to the best allowed AprilTag. */
  public Optional<Double> getYawToBestTagRad(int... allowedIds) {
    PhotonPipelineResult result = camera.getLatestResult();
    if (!result.hasTargets()) return Optional.empty();

    PhotonTrackedTarget best = null;
    double bestAbsYawDeg = 1e9;

    for (PhotonTrackedTarget t : result.getTargets()) {
      int id = t.getFiducialId();
      boolean allowed = false;
      for (int a : allowedIds) {
        if (id == a) { allowed = true; break; }
      }
      if (!allowed) continue;

      double yawDeg = t.getYaw(); // +deg means target is to the right in PhotonVision
      double abs = Math.abs(yawDeg);

      if (abs < bestAbsYawDeg) {
        bestAbsYawDeg = abs;
        best = t;
      }
    }

    if (best == null) return Optional.empty();
    return Optional.of(Units.degreesToRadians(best.getYaw()));
  }

  /** Dynamic measurement uncertainty for Kalman fusion. */
  public Matrix<N3, N1> getEstimationStdDevs(EstimatedRobotPose est) {
    int tagCount = est.targetsUsed.size();

    double baseXY =
        (tagCount >= 2)
            ? VisionConstants.kBaseXYStdDevMultiTagMeters
            : VisionConstants.kBaseXYStdDevSingleTagMeters;

    double baseTheta =
        (tagCount >= 2)
            ? VisionConstants.kBaseThetaStdDevMultiTagRad
            : VisionConstants.kBaseThetaStdDevSingleTagRad;

    double avgDist = averageDistanceMeters(est.targetsUsed);

    double distanceScale = 1.0 + (avgDist * avgDist) / 4.0;
    double tagScale = (tagCount >= 2) ? (2.0 / tagCount) : 1.0;

    double xy = baseXY * distanceScale * tagScale;
    double theta = baseTheta * distanceScale * tagScale;

    return VecBuilder.fill(xy, xy, theta);
  }

  private boolean isGoodEstimate(EstimatedRobotPose est) {
    List<PhotonTrackedTarget> targets = est.targetsUsed;
    if (targets.isEmpty()) return false;

    double avgDist = averageDistanceMeters(targets);
    if (avgDist > VisionConstants.kMaxTargetDistanceMeters) return false;

    if (targets.size() == 1) {
      double ambiguity = targets.get(0).getPoseAmbiguity();
      if (ambiguity < 0.0) return false;
      if (ambiguity > VisionConstants.kMaxPoseAmbiguity) return false;
    }

    return true;
  }

  private double averageDistanceMeters(List<PhotonTrackedTarget> targets) {
    double sum = 0.0;
    for (PhotonTrackedTarget t : targets) {
      sum += t.getBestCameraToTarget().getTranslation().getNorm();
    }
    return sum / targets.size();
  }
}
