package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * PhotonVision subsystem that reads targets (including AprilTags) from a PhotonVision
 * server (can be running on an Orange Pi). Provides helpers to get distance,
 * yaw/pitch, and motion-compensated aiming while moving.
 *
 * Notes / assumptions:
 * - PhotonVision must be publishing a camera with the provided camera name.
 * - If PhotonVision runs on an Orange Pi and uses a separate NetworkTables server,
 *   call startOrangePiClient(host) early in robot initialization.
 * - Motion compensation uses the camera->target translation reported by PhotonVision.
 *   Translation axes assumed: X forward, Y left, Z up. The yaw is computed from
 *   atan2(lateral, forward) and converted to degrees.
 */
public class PhotonVisionSubsytem extends SubsystemBase {

	private final PhotonCamera camera;
	private PhotonPipelineResult latestResult = null;

	// default latency to account for processing / network (seconds)
	private double defaultLatencySeconds = 0.05;

	public PhotonVisionSubsytem(String cameraName) {
		this.camera = new PhotonCamera(cameraName);
	}

	/**
	 * If your Orange Pi is running a NetworkTables server (or is the PhotonVision server),
	 * call this to make the robot connect to that server so PhotonVision entries are reachable.
	 * Example: startOrangePiClient("192.168.1.42");
	 */
	public void startOrangePiClient(String ipAddress) {
		// Uses reflection to attempt to call startClient(String...) on NetworkTableInstance.
		// Some WPILib versions expose startClient(...) while others may not; reflection
		// avoids a direct compile dependency on that method.
		try {
			NetworkTableInstance nti = NetworkTableInstance.getDefault();
			java.lang.reflect.Method m = nti.getClass().getMethod("startClient", String[].class);
			// invoke expects an Object[] for varargs when passing a single String[]
			m.invoke(nti, (Object) new String[] { ipAddress });
			SmartDashboard.putString("Photon/OrangePiClient", "Connecting to " + ipAddress);
		} catch (Exception e) {
			// If reflection fails, surface a message so the user can manually connect NT client.
			SmartDashboard.putString("Photon/OrangePiClient", "startClient unavailable - connect manually: " + ipAddress);
		}
	}

	@Override
	public void periodic() {
		// update cached result and publish a few useful values for debugging / dashboard
		latestResult = camera.getLatestResult();

		boolean hasTarget = (latestResult != null && latestResult.hasTargets());
		SmartDashboard.putBoolean("Photon/HasTarget", hasTarget);

		if (hasTarget) {
			PhotonTrackedTarget t = latestResult.getBestTarget();
			Optional<Transform3d> maybe = Optional.ofNullable(t.getBestCameraToTarget());
			if (maybe.isPresent()) {
				Transform3d tx = maybe.get();
				double distance = horizontalDistance(tx);
				SmartDashboard.putNumber("Photon/DistanceMeters", distance);
			}
			SmartDashboard.putNumber("Photon/TargetYawDeg", t.getYaw());
			SmartDashboard.putNumber("Photon/TargetPitchDeg", t.getPitch());
			SmartDashboard.putNumber("Photon/TargetID", t.getFiducialId());
		}
	}

	/**
	 * Returns whether a target is currently visible.
	 */
	public boolean hasTarget() {
		return latestResult != null && latestResult.hasTargets();
	}

	/**
	 * Returns the best tracked target if present.
	 */
	public Optional<PhotonTrackedTarget> getBestTarget() {
		if (!hasTarget()) {
			return Optional.empty();
		}
		return Optional.of(latestResult.getBestTarget());
	}

	/**
	 * Returns the camera-to-target Transform3d if available.
	 */
	public Optional<Transform3d> getCameraToTarget() {
		return getBestTarget().map(PhotonTrackedTarget::getBestCameraToTarget);
	}

	/**
	 * Horizontal distance (meters) computed from the camera->target transform.
	 * This ignores vertical component and returns sqrt(x^2 + y^2).
	 */
	public double getDistanceMeters() {
		return getCameraToTarget()
				.map(PhotonVisionSubsytem::horizontalDistance)
				.orElse(Double.NaN);
	}

	/**
	 * Returns the raw yaw (degrees) reported by PhotonVision for the best target.
	 */
	public double getYawDeg() {
		return getBestTarget().map(PhotonTrackedTarget::getYaw).orElse(Double.NaN);
	}

	/**
	 * Returns raw pitch (degrees) reported by PhotonVision for the best target.
	 */
	public double getPitchDeg() {
		return getBestTarget().map(PhotonTrackedTarget::getPitch).orElse(Double.NaN);
	}

	/**
	 * Compute an adjusted yaw (degrees) compensating for robot motion during latency.
	 * vx: forward velocity in meters/second (positive forward)
	 * vy: lateral velocity in meters/second (positive left)
	 * latencySeconds: time to compensate (processing + network). If not set, uses defaultLatencySeconds.
	 *
	 * Algorithm: predict camera->target translation after robot moves for latencySeconds,
	 * then compute yaw = atan2(predictedY, predictedX) in degrees.
	 */
	public double getAdjustedYawForMotion(double vx, double vy, double latencySeconds) {
		if (!hasTarget()) {
			return Double.NaN;
		}

		Transform3d tx = getCameraToTarget().get();
		Translation3d t = tx.getTranslation();

		// predict change in camera->target due to robot movement
		double predictedX = t.getX() - vx * latencySeconds;
		double predictedY = t.getY() - vy * latencySeconds;

		// yaw is angle between forward (X) and lateral (Y)
		double yawRad = Math.atan2(predictedY, predictedX);
		return Math.toDegrees(yawRad);
	}

	/**
	 * Convenience overload using default latency.
	 */
	public double getAdjustedYawForMotion(double vx, double vy) {
		return getAdjustedYawForMotion(vx, vy, defaultLatencySeconds);
	}

	/**
	 * Set the default latency used for motion compensation (seconds).
	 */
	public void setDefaultLatencySeconds(double latencySeconds) {
		this.defaultLatencySeconds = latencySeconds;
	}

	private static double horizontalDistance(Transform3d tx) {
		Translation3d t = tx.getTranslation();
		return Math.hypot(t.getX(), t.getY());
	}

}


