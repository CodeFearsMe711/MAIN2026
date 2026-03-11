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

UsbCamera camera = CameraServer.startAutomaticCapture(0);
camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
camera.setPixelFormat(PixelFormat.kYUYV);
camera.setResolution(320, 240);
camera.setFPS(20);
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
// Publish LED diagnostics and allow reinit from dashboard
    if (leds != null) {
      SmartDashboard.putString("LED Status", leds.getStatus());
      SmartDashboard.putBoolean("LED Initialized", leds.isInitialized());
      SmartDashboard.putNumber("LED Port (cfg)", leds.getConfiguredPort());
      SmartDashboard.putNumber("LED Length (cfg)", leds.getConfiguredLength());
      SmartDashboard.putNumber("LED Active Port", leds.getActivePort());
      SmartDashboard.putString("LED Last Error", leds.getLastError() == null ? "none" : leds.getLastError());
    } else {
      SmartDashboard.putString("LED Status", "Not constructed");
      SmartDashboard.putBoolean("LED Initialized", false);
      SmartDashboard.putNumber("LED Port (cfg)", -1);
      SmartDashboard.putNumber("LED Length (cfg)", -1);
      SmartDashboard.putNumber("LED Active Port", -1);
      SmartDashboard.putString("LED Last Error", "no-led-object");
    }


    // Dashboard button to attempt reinit (set true in Shuffleboard/SmartDashboard)
    boolean doReinit = SmartDashboard.getBoolean("LED Reinit", false);
    if (doReinit) {
      if (leds == null) {
        try {
          leds = new LumenLightsSubsystem();
        } catch (Exception e) {
          DriverStation.reportError("Failed to construct LEDs during reinit: " + e.getMessage(), e.getStackTrace());
        }
      }
      if (leds != null) {
        leds.reinit();
      }
      // reset the dashboard toggle so operator doesn't continuously spam it
      SmartDashboard.putBoolean("LED Reinit", false);
    }


    // Diagnostic manual test: attempt to set red and record success/failure.
    if (SmartDashboard.getBoolean("LED Force Red", false)) {
      String last = "no-led-object";
      if (leds != null) {
        try {
          leds.setAllRGB(255, 0, 0);
          last = "test-set-red:SUCCESS";
        } catch (Exception e) {
          last = "test-set-red:FAILED: " + e.getMessage();
          DriverStation.reportError("LED test-set-red failed: " + e.getMessage(), e.getStackTrace());
        }
        // if success, keep the color but caller can disable the toggle later
      }
      SmartDashboard.putString("LED Last Test", last);
    }


    // New: dashboard-triggered cycle test to flash RGB for visual debugging
    boolean doCycle = SmartDashboard.getBoolean("LED Cycle Test", false);
    if (doCycle) {
      if (leds != null) {
        leds.cycleTest(5); // flash 5 cycles
        SmartDashboard.putString("LED Last Cycle", "started");
      } else {
        SmartDashboard.putString("LED Last Cycle", "no-led-object");
      }
      SmartDashboard.putBoolean("LED Cycle Test", false);
    }


    // Dashboard button to attempt port scan (set true in Shuffleboard/SmartDashboard)
    boolean doPortScan = SmartDashboard.getBoolean("LED Port Scan", false);
    if (doPortScan) {
      if (leds == null) {
        try {
          leds = new LumenLightsSubsystem();
        } catch (Exception e) {
          DriverStation.reportError("Failed to construct LEDs for scan: " + e.getMessage(), e.getStackTrace());
        }
      }
      if (leds != null) {
        // scan ports 0..12 (adjust range as needed for your breakout)
        leds.scanPortsAsync(12);
      }
      SmartDashboard.putBoolean("LED Port Scan", false);
    }



  } catch (Exception e) {
    DriverStation.reportError("robotPeriodic exception: " + e.getMessage(), e.getStackTrace());
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