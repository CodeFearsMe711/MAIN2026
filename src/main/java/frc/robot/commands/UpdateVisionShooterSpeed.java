package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.SWERVE.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.Vision.PhotonVisionSubsytem;

public class UpdateVisionShooterSpeed extends Command {

  private final PhotonVisionSubsytem vision;
  private final ShooterSubsystem shooter;
  private final CommandSwerveDrivetrain drivetrain;

  public UpdateVisionShooterSpeed(
      PhotonVisionSubsytem vision,
      ShooterSubsystem shooter,
      CommandSwerveDrivetrain drivetrain) {
    this.vision = vision;
    this.shooter = shooter;
    this.drivetrain = drivetrain;

    addRequirements(shooter);
  }

  private static double distanceToShooterRps(double distanceMeters) {
    if (distanceMeters < 2.0) return 70.0;
    if (distanceMeters < 3.0) return 90.0;
    if (distanceMeters < 4.0) return 110.0;
    if (distanceMeters < 5.0) return 125.0;
    return 140.0;
  }

  @Override
  public void execute() {
    Optional<Double> distanceOpt =
        vision.getBestAllowedTagDistanceMeters(
            drivetrain.getState().Pose,
            VisionConstants.kAimTagIds);

    if (distanceOpt.isEmpty()) {
      return;
    }

    double shooterRps = distanceToShooterRps(distanceOpt.get());
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