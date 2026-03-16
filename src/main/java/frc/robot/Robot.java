package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.reflect.Method;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDS.LumenLightsSubsystem;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.commands.Intake.IntakeArmEnableDropCommand;

import frc.robot.subsystems.LEDS.ConnectorXLeds;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private LumenLightsSubsystem leds;
  private final RobotContainer m_robotContainer;

  private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
      .withTimestampReplay()
      .withJoystickReplay();

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // Try to reuse a lights instance from RobotContainer if it provides one, otherwise construct.
    try {
      // Reflection: if RobotContainer has getLumenLightsSubsystem(), use it.
      try {
        Method m = m_robotContainer.getClass().getMethod("getLumenLightsSubsystem");
        Object obj = m.invoke(m_robotContainer);
        if (obj instanceof LumenLightsSubsystem) {
          leds = (LumenLightsSubsystem) obj;
        }
      } catch (NoSuchMethodException nsme) {
        // RobotContainer doesn't expose lights; we'll construct our own below.
      }


      if (leds == null) {
        leds = new LumenLightsSubsystem();
      }


      if (leds != null && leds.isInitialized()) {
        leds.setAllRGB(255, 0, 0);
        SmartDashboard.putString("LED Status", "OK - set red (port " + leds.getActivePort() + ")");
      } else if (leds != null) {
        SmartDashboard.putString("LED Status", leds.getStatus());
        // If we failed to init, log the scan result (if any) to DriverStation for immediate visibility
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

    // Boot-only zero: happens before any commands/autos run
    m_robotContainer.getIntakeArmSubsystem().zeroArmPositionOnBoot();

//UsbCamera camera = CameraServer.startAutomaticCapture(0);
//camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
//camera.setPixelFormat(PixelFormat.kYUYV);
//camera.setResolution(320, 240);
//camera.setFPS(20);
}
  // Robot.java

private double m_lastDashTime = 0.0;
private double m_lastHeavyTime = 0.0;

@Override
public void robotPeriodic() {
  CommandScheduler.getInstance().run();
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
    // Ensure LEDs are stopped when disabled
    if (leds != null) {
      leds.stop();
    }
  }



  @Override
public void autonomousInit() {
  m_autonomousCommand = m_robotContainer.getAutonomousCommand();

  if (m_autonomousCommand != null) {
    m_autonomousCommand.schedule();
  }
}
}