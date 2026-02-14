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
                Units.inchesToMeters(10.0), // forward
                Units.inchesToMeters(0.0),  // left
                Units.inchesToMeters(20.0)  // up
            ),
            new Rotation3d(
                0.0,                        // roll
                Units.degreesToRadians(0.0),// pitch
                Units.degreesToRadians(0.0) // yaw
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
    public static final int[] kAimTagIds = { 26, 18, 21 }; // TODO: replace with your actual tag IDs

    // PID gains for turning to face the tag using yaw error
    public static final double kAimKp = 4.0;
    public static final double kAimKi = 0.0;
    public static final double kAimKd = 0.25;

    // Clamp rotational command (rad/s)
    public static final double kAimMaxOmegaRadPerSec = Units.degreesToRadians(360.0);

    // Deadband for small yaw errors
    public static final double kAimMinErrorRad = Units.degreesToRadians(1.5);
  }
}
