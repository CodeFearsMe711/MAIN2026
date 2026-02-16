package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.SWERVE.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.PhotonVisionSubsytem;

public class AimHubTagOverride extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final PhotonVisionSubsytem vision;
  private final PIDController pid;

  public AimHubTagOverride(CommandSwerveDrivetrain drivetrain, PhotonVisionSubsytem vision, PIDController pid) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.pid = pid;
    // IMPORTANT: do NOT addRequirements(drivetrain) or you’ll cancel the path follower
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
    var yawOpt = vision.getYawToBestTagRad(VisionConstants.kAimTagIds);

    if (yawOpt.isEmpty()) {
      drivetrain.clearOmegaOverride();
      pid.reset();
      return;
    }

    double yawErr = yawOpt.get(); // rad; + means tag to the right (based on your existing usage)
    if (Math.abs(yawErr) < VisionConstants.kAimMinErrorRad) {
      drivetrain.setOmegaOverride(0.0);
      return;
    }

    double cmd = pid.calculate(yawErr, 0.0);
    double omega = clamp(-cmd, -VisionConstants.kAimMaxOmegaRadPerSec, VisionConstants.kAimMaxOmegaRadPerSec);

    drivetrain.setOmegaOverride(omega);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.clearOmegaOverride();
    pid.reset();
  }

  @Override
  public boolean isFinished() {
    return false; // use it as a zoned/while marker in PathPlanner
  }
}
