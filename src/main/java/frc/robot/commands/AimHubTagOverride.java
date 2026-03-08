package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.SWERVE.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.PhotonVisionSubsytem;
import frc.robot.util.AimAssistMath;

public class AimHubTagOverride extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonVisionSubsytem vision;
  private final PIDController pid;

  public AimHubTagOverride(
      CommandSwerveDrivetrain drivetrain,
      PhotonVisionSubsytem vision,
      PIDController pid) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.pid = pid;

    // Do NOT addRequirements(drivetrain) or it will fight PathPlanner
  }

  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  @Override
  public void initialize() {
    pid.reset();
  }

  @Override
  public void execute() {
    var result = vision.getLatestResult();
    if (!result.hasTargets()) {
      drivetrain.clearOmegaOverride();
      pid.reset();
      return;
    }

    var bestAllowed =
        AimAssistMath.findBestAllowedTarget(result.getTargets(), VisionConstants.kAimTagIds);

    if (bestAllowed == null) {
      drivetrain.clearOmegaOverride();
      pid.reset();
      return;
    }

    ChassisSpeeds speeds = drivetrain.getRobotRelativeSpeeds();
    var yawOpt = AimAssistMath.getCorrectedYawRad(bestAllowed, speeds);

    if (yawOpt.isEmpty()) {
      drivetrain.clearOmegaOverride();
      pid.reset();
      return;
    }

    double yawErrRad = yawOpt.get();

    if (Math.abs(yawErrRad) < VisionConstants.kAimMinErrorRad) {
      drivetrain.setOmegaOverride(0.0);
      return;
    }

    double cmd = pid.calculate(yawErrRad, 0.0);
    double omega =
        clamp(
            cmd,
            -VisionConstants.kAimMaxOmegaRadPerSec,
            VisionConstants.kAimMaxOmegaRadPerSec);

    drivetrain.setOmegaOverride(omega);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.clearOmegaOverride();
    pid.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}