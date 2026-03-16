package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.reflect.Method;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDS.LumenLightsSubsystem;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private LumenLightsSubsystem leds;
  private final RobotContainer m_robotContainer;

  @SuppressWarnings("unused")
  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
      .withTimestampReplay()
      .withJoystickReplay();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", "MAIN2026-Post-Great-Northern");
    Logger.recordMetadata("Robot", "Competition");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      Logger.addDataReceiver(new WPILOGWriter(""));
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    try {
      try {
        Method m = m_robotContainer.getClass().getMethod("getLumenLightsSubsystem");
        Object obj = m.invoke(m_robotContainer);
        if (obj instanceof LumenLightsSubsystem) {
          leds = (LumenLightsSubsystem) obj;
        }
      } catch (NoSuchMethodException nsme) {
        // RobotContainer does not expose lights in this build.
      }

      if (leds == null) {
        leds = new LumenLightsSubsystem();
      }

      if (leds != null && leds.isInitialized()) {
        leds.setAllRGB(255, 0, 0);
        SmartDashboard.putString("LED Status", "OK - set red (port " + leds.getActivePort() + ")");
      } else if (leds != null) {
        SmartDashboard.putString("LED Status", leds.getStatus());
        String scan = leds.getLastScanResult();
        if (scan != null && !scan.isEmpty()) {
          DriverStation.reportWarning("LED init failed; auto-probe:\n" + scan, false);
        }
      } else {
        SmartDashboard.putString("LED Status", "null");
      }
    } catch (Exception e) {
      DriverStation.reportError("Failed to init LumenLightsSubsystem: " + e.getMessage(), e.getStackTrace());
      leds = null;
      SmartDashboard.putString("LED Status", "Init error: " + e.getMessage());
    }

    m_robotContainer.getIntakeArmSubsystem().zeroArmPositionOnBoot();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.logAdvantageKit();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }

    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledInit() {
    if (leds != null) {
      leds.stop();
    }
  }
}
