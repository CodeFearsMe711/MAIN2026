// src/main/java/frc/robot/Constants/Constants.java
package frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int cDriverControllerPort = 1;
  }

  /** LED related constants */
  public static class Lights {
    public static final int kLedPort = 9;
    public static final int kLedLength = 60;
  }

  /** Vision / AprilTag constants */
  public static class VisionConstants {
    // PhotonVision camera name (must match PhotonVision UI)
    public static final String kCameraName = "Photon_Camera";

    // Robot -> Camera transform (MEASURE AND UPDATE)
    // +X forward, +Y left, +Z up
    public static final Transform3d kRobotToCamera =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-5.0), // forward
                Units.inchesToMeters(-11.0),  // left
                Units.inchesToMeters(19.5)  // up
            ),
            new Rotation3d(
                0.0,                        // roll
                Units.degreesToRadians(17.6),// pitch
                Units.degreesToRadians(0) // yaw
            )
        );

    // Filters for pose estimation
    public static final double kMaxPoseAmbiguity = 0.20;
    public static final double kMaxTargetDistanceMeters = 6.0;

    // Pose fusion std dev bases
    public static final double kBaseXYStdDevSingleTagMeters = 1.25;
    public static final double kBaseThetaStdDevSingleTagRad = Units.degreesToRadians(12.0);
    public static final double kBaseXYStdDevMultiTagMeters = 0.35;
    public static final double kBaseThetaStdDevMultiTagRad = Units.degreesToRadians(4.0);

    // =========================
    // Hold-to-aim (teleop) settings
    // =========================

    // Put the "right" tag IDs you want to aim at here
    public static final int[] kAimTagIds = { 10, 26, 21, 18, 5, 2}; 

    // PID gains for turning to face the tag using yaw error
    public static final double kAimKp = 5;
    public static final double kAimKi = 0.0;
    public static final double kAimKd = 0.25;

    // Clamp rotational command (rad/s)
    public static final double kAimMaxOmegaRadPerSec = Units.degreesToRadians(360.0);

    // Deadband for small yaw errors
    public static final double kAimMinErrorRad = Units.degreesToRadians(1.5);

    // Aiming-only yaw trim (deg). + means pretend target is more to the right.
    public static final double kAimYawOffsetDeg = 10.0;

    // =========================
    // Aim Assist: RANGE HOLD (teleop)
    // =========================

    // Desired distance away from the hub/tag (meters)
    public static final double kAimTargetDistanceMeters = 1.5;

    // PID gains for distance hold (camera-to-tag distance)
    public static final double kAimRangeKp = 0.1;
    public static final double kAimRangeKi = 0.0;
    public static final double kAimRangeKd = 0.0;

    // Clamp for automatic in/out correction while aiming (m/s)
    public static final double kAimMaxRangeSpeedMps = 1.25;

    // Deadband around the target distance (meters)
    public static final double kAimRangeDeadbandMeters = 0.10;

    // =========================
    // Shoot-on-the-move lead compensation
    // =========================
    public static final double kAimFlightTimeSec = 0.35;            // start ~0.30–0.45
    public static final double kAimMaxLeadDeg = 8.0;                // clamp
    public static final double kAimLeadScale = 1.0;                 // 1.0 = pure physics, tune 0.6–1.4
  }
}