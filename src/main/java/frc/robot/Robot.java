package frc.robot;

import java.lang.reflect.Method;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LEDS.LumenLightsSubsystem;

public class Robot extends TimedRobot {
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
    try {
      try {
        Method m = m_robotContainer.getClass().getMethod("getLumenLightsSubsystem");
        Object obj = m.invoke(m_robotContainer);
        if (obj instanceof LumenLightsSubsystem) {
          leds = (LumenLightsSubsystem) obj;
        }
      } catch (NoSuchMethodException ignored) {
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

    UsbCamera camera = CameraServer.startAutomaticCapture(0);
    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    camera.setPixelFormat(PixelFormat.kYUYV);
    camera.setResolution(320, 240);
    camera.setFPS(20);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    m_robotContainer.updateBrownoutHeadingCapture(
        RobotController.getBatteryVoltage(),
        RobotController.isBrownedOut());
    m_robotContainer.persistHeadingForBrownoutRecovery();
    m_robotContainer.updateVisionFusion();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }

    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.restoreHeadingAfterBrownout();
    m_robotContainer.getDrivetrain().seedFieldCentric();
  }

  @Override
  public void disabledInit() {
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
