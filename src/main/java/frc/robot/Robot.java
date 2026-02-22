package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.LEDS.ConnectorXLeds;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private ConnectorXLeds leds;
  private final RobotContainer m_robotContainer;

  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
      .withTimestampReplay()
      .withJoystickReplay();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    leds = new ConnectorXLeds();
    leds.start();

    // Boot-only zero: happens before any commands/autos run
    m_robotContainer.getIntakeArmSubsystem().zeroArmPositionOnBoot();
  }

  // Robot.java

private double m_lastDashTime = 0.0;
private double m_lastHeavyTime = 0.0;

@Override
public void robotPeriodic() {
  try {
    // Always run the scheduler first, every loop
    CommandScheduler.getInstance().run();

    final double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    // Fast dashboard (10 Hz)
    if ((now - m_lastDashTime) >= 0.10) {
      m_lastDashTime = now;

      SmartDashboard.putNumber("Test", 42);

      if (m_robotContainer != null) {
        SmartDashboard.putNumber(
            "Arm Degrees",
            m_robotContainer.getIntakeArmSubsystem().getDegrees()
        );

        if (m_robotContainer.getDrivetrain() != null) {
          Pose2d pose = m_robotContainer.getDrivetrain().getState().Pose;
          SmartDashboard.putNumber("Odo/X_m", pose.getX());
          SmartDashboard.putNumber("Odo/Y_m", pose.getY());
          SmartDashboard.putNumber("Odo/Heading_deg", pose.getRotation().getDegrees());
        }
      }
    }

    // Heavy work (5 Hz) — vision fusion / hub status / replay updates
    if ((now - m_lastHeavyTime) >= 0.20) {
      m_lastHeavyTime = now;

      if (m_timeAndJoystickReplay != null) {
        m_timeAndJoystickReplay.update();
      }

      if (m_robotContainer != null) {
        m_robotContainer.updateVisionFusion();
        m_robotContainer.publishMatchHubStatus();
      }
    }

  } catch (Exception e) {
    DriverStation.reportError("robotPeriodic exception: " + e.getMessage(), e.getStackTrace());
  }
}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancelAll();
    }
  }
}