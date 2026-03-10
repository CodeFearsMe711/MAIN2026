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
  }

  @Override
  public void execute() {
    var result = vision.getLatestResult();

    if (!result.hasTargets()) {
      SmartDashboard.putBoolean("AutoAim/HasAllowedTarget", false);
      return;
    }

    var bestAllowed =
        AimAssistMath.findBestAllowedTarget(
            result.getTargets(),
            VisionConstants.kAimTagIds);

    if (bestAllowed == null) {
      SmartDashboard.putBoolean("AutoAim/HasAllowedTarget", false);
      return;
    }

    SmartDashboard.putBoolean("AutoAim/HasAllowedTarget", true);

    double distanceMeters = AimAssistMath.getDistanceMeters(bestAllowed);
    double shooterRps = ShooterMath.distanceMetersToShooterRps(distanceMeters);

    SmartDashboard.putNumber("AutoAim/DistanceMeters", distanceMeters);
    SmartDashboard.putNumber("AutoAim/ShooterTargetRPS", shooterRps);

    shooter.updateVisionTargetRPS(shooterRps);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}