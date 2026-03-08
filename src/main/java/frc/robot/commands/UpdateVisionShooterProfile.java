package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.subsystems.Vision.PhotonVisionSubsytem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.util.AimAssistMath;
import frc.robot.util.ShooterMath;

public class UpdateVisionShooterProfile extends Command {
  private final PhotonVisionSubsytem vision;
  private final ShooterSubsystem shooter;

  public UpdateVisionShooterProfile(
      PhotonVisionSubsytem vision,
      ShooterSubsystem shooter) {
    this.vision = vision;
    this.shooter = shooter;

    // Only add this if you want this command to own the shooter.
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

    shooter.setTargetRPS(shooterRps);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}