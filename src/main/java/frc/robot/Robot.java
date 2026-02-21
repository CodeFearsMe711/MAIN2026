// src/main/java/frc/robot/Robot.java
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
  }

  @Override
  public void robotPeriodic() {
    try {
      SmartDashboard.putNumber("Test", 42);

      m_timeAndJoystickReplay.update();

      m_robotContainer.updateVisionFusion();
      m_robotContainer.publishMatchHubStatus();

      SmartDashboard.putNumber("Arm Degrees", m_robotContainer.getIntakeArmSubsystem().getDegrees());

      CommandScheduler.getInstance().run();

      if (m_robotContainer != null && m_robotContainer.getDrivetrain() != null) {
        Pose2d pose = m_robotContainer.getDrivetrain().getState().Pose;

        SmartDashboard.putNumber("Odo/X_m", pose.getX());
        SmartDashboard.putNumber("Odo/Y_m", pose.getY());
        SmartDashboard.putNumber("Odo/Heading_deg", pose.getRotation().getDegrees());
      }
    } catch (Exception e) {
      DriverStation.reportError("robotPeriodic exception: " + e.getMessage(), e.getStackTrace());
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancelAll();
    }
  }
}