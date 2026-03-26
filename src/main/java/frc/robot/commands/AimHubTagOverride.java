package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.SWERVE.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.PhotonVisionSubsytem;
import frc.robot.util.AimAssistMath;
import frc.robot.util.ShooterMath;

public class AimHubTagOverride extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonVisionSubsytem vision;
  private final ShooterSubsystem shooter;
  private final PIDController pid;

  private double lastOmega = 0.0;
  private int lockedAimTagId = -1;
  private int lockedShooterTagId = -1;
  private double filteredYawErrRad = 0.0;
  private boolean hasFilteredYawErr = false;
  private static final double kYawErrorFilterAlpha = 0.35;

  public AimHubTagOverride(
      CommandSwerveDrivetrain drivetrain,
      PhotonVisionSubsytem vision,
      ShooterSubsystem shooter,
      PIDController pid) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.shooter = shooter;
    this.pid = pid;
  }

  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  @Override
  public void initialize() {
    pid.reset();
    lastOmega = 0.0;
    lockedAimTagId = -1;
    lockedShooterTagId = -1;
    hasFilteredYawErr = false;
    shooter.setVisionEnabled(true);
    SmartDashboard.putBoolean("AutoAimLBActive", true);
  }

  @Override
  public void execute() {
    var result = vision.getLatestResult();

    if (!result.hasTargets()) {
      SmartDashboard.putBoolean("AutoAimHasAllowedTarget", false);
      drivetrain.clearOmegaOverride();
      pid.reset();
      lastOmega = 0.0;
      lockedAimTagId = -1;
      lockedShooterTagId = -1;
      hasFilteredYawErr = false;
      return;
    }

    var bestAimTarget =
    AimAssistMath.findBestAllowedTarget(
        result.getTargets(),
        lockedAimTagId,
        VisionConstants.kAimTagIds);

var bestShooterTarget =
    AimAssistMath.findBestAllowedTarget(
        result.getTargets(),
        lockedShooterTagId,
        VisionConstants.kShooterTagIds);

    if (bestAimTarget == null) {
  SmartDashboard.putBoolean("AutoAimHasAllowedTarget", false);
  drivetrain.clearOmegaOverride();
  pid.reset();
  lastOmega = 0.0;
  lockedAimTagId = -1;
  lockedShooterTagId = -1;
  hasFilteredYawErr = false;
  return;
}

lockedAimTagId = bestAimTarget.getFiducialId();
SmartDashboard.putBoolean("AutoAimHasAllowedTarget", true);

if (bestShooterTarget != null) {
  lockedShooterTagId = bestShooterTarget.getFiducialId();
  double area = bestShooterTarget.getArea();
  double shooterSpeed = ShooterMath.tagAreaToShooterRps(area);

  SmartDashboard.putNumber("AutoAimArea", area);
  SmartDashboard.putNumber("AutoAimShooterTargetRPS", shooterSpeed);

  shooter.updateVisionSpeed(shooterSpeed);
}

double yawDeg = bestAimTarget.getYaw();
SmartDashboard.putNumber("AutoAimYawDeg", yawDeg);

ChassisSpeeds speeds = drivetrain.getRobotRelativeSpeeds();
var yawOpt = AimAssistMath.getCorrectedYawRad(bestAimTarget, speeds);
    if (yawOpt.isEmpty()) {
      drivetrain.clearOmegaOverride();
      pid.reset();
      lastOmega = 0.0;
      return;
    }

    double yawErrRadRaw = yawOpt.get();
    if (!hasFilteredYawErr) {
      filteredYawErrRad = yawErrRadRaw;
      hasFilteredYawErr = true;
    } else {
      filteredYawErrRad =
          (kYawErrorFilterAlpha * yawErrRadRaw)
              + ((1.0 - kYawErrorFilterAlpha) * filteredYawErrRad);
    }
    double yawErrRad = filteredYawErrRad;
    double omega;

    if (Math.abs(yawErrRad) < VisionConstants.kAimMinErrorRad) {
      omega = 0.0;
      pid.reset();
    } else {
      double cmd = pid.calculate(yawErrRad, 0.0);
      double unclampedOmega =
          clamp(
              cmd,
              -VisionConstants.kAimMaxOmegaRadPerSec,
              VisionConstants.kAimMaxOmegaRadPerSec);

      omega = 0.75 * lastOmega + 0.25 * unclampedOmega;
    }

    lastOmega = omega;

    SmartDashboard.putNumber(
        "AutoAimYawErrDeg",
        edu.wpi.first.math.util.Units.radiansToDegrees(yawErrRad));
    SmartDashboard.putNumber(
        "AutoAimYawErrRawDeg",
        edu.wpi.first.math.util.Units.radiansToDegrees(yawErrRadRaw));
    SmartDashboard.putNumber("AutoAimOmegaCmd", omega);

    drivetrain.setOmegaOverride(omega);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("AutoAimLBActive", false);
    SmartDashboard.putBoolean("AutoAimHasAllowedTarget", false);
    drivetrain.clearOmegaOverride();
    pid.reset();
    lastOmega = 0.0;
    lockedAimTagId = -1;
    lockedShooterTagId = -1;
    hasFilteredYawErr = false;
    shooter.setVisionEnabled(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
