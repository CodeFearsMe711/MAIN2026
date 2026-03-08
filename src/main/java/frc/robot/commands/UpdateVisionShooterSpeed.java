package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.PhotonVisionSubsytem;
import frc.robot.util.AimAssistMath;
import frc.robot.util.ShooterMath;

public class UpdateVisionShooterSpeed extends Command {
  private final PhotonVisionSubsytem vision;
  private final ShooterSubsystem shooter;

  public UpdateVisionShooterSpeed(
      PhotonVisionSubsytem vision,
      ShooterSubsystem shooter) {
    this.vision = vision;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  @Override
  public void execute() {
    var result = vision.getLatestResult();

    if (!result.hasTargets()) {
      return;
    }

    var bestAllowed =
        AimAssistMath.findBestAllowedTarget(
            result.getTargets(),
            VisionConstants.kAimTagIds);

    if (bestAllowed == null) {
      return;
    }

    double distanceMeters = AimAssistMath.getDistanceMeters(bestAllowed);
    double shooterRps = ShooterMath.distanceMetersToShooterRps(distanceMeters);

    SmartDashboard.putNumber("Aim/DistanceMeters", distanceMeters);
    SmartDashboard.putNumber("Aim/VisionShooterRPS", shooterRps);

    // IMPORTANT: direct velocity command so it changes immediately
    shooter.setRPS(shooterRps);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}