// src/main/java/frc/robot/Robot.java
package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
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
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();

    // Vision pose fusion (with outlier rejection)
    m_robotContainer.updateVisionFusion();

    m_robotContainer.publishMatchHubStatus();

    CommandScheduler.getInstance().run();
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
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void robotInit() {
    leds = new ConnectorXLeds();
    leds.start();
  }
}
