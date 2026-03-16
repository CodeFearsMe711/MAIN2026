package frc.robot.commands.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.PhotonVisionSubsytem;
import frc.robot.util.AimAssistMath;
import frc.robot.util.ShooterMath;

public class NamedShooter extends Command {
  private final ShooterSubsystem shooter;
  private final PhotonVisionSubsytem vision;

  private static final double FALLBACK_RPS = 65.0; // <-- change this number

  public NamedShooter(ShooterSubsystem shooter, PhotonVisionSubsytem vision) {
    this.shooter = shooter;
    this.vision = vision;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setVisionEnabled(true);
    shooter.updateVisionSpeed(FALLBACK_RPS);
  }

  @Override
  public void execute() {
    var result = vision.getLatestResult();

    if (!result.hasTargets()) {
      shooter.updateVisionSpeed(FALLBACK_RPS);
      return;
    }

    var bestAllowed =
        AimAssistMath.findBestAllowedTarget(
            result.getTargets(),
            VisionConstants.kShooterTagIds);

    if (bestAllowed == null) {
      shooter.updateVisionSpeed(FALLBACK_RPS);
      return;
    }

    double area = bestAllowed.getArea();
    double targetRPS = ShooterMath.tagAreaToShooterRps(area);

    shooter.updateVisionSpeed(targetRPS);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setVisionEnabled(false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}