// src/main/java/frc/robot/Robot.java
package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private void seedHeadingForAlliance() {
    if (m_robotContainer == null || m_robotContainer.getDrivetrain() == null) return;

    var alliance = DriverStation.getAlliance();
    boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

    Pose2d currentPose = m_robotContainer.getDrivetrain().getState().Pose;

    m_robotContainer.getDrivetrain().resetPose(
      new Pose2d(
        currentPose.getTranslation(),
        isRed ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0)
      )
    );
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

      // Vision pose fusion (with outlier rejection)
      m_robotContainer.updateVisionFusion();
      m_robotContainer.publishMatchHubStatus();

      // Always publish arm angle so the key never "disappears"
      SmartDashboard.putNumber("Arm Degrees", m_robotContainer.getIntakeArmSubsystem().getDegrees());

      CommandScheduler.getInstance().run();

      if (m_robotContainer != null && m_robotContainer.getDrivetrain() != null) {
        Pose2d pose = m_robotContainer.getDrivetrain().getState().Pose;

        SmartDashboard.putNumber("Odo/X_m", pose.getX());
        SmartDashboard.putNumber("Odo/Y_m", pose.getY());
        SmartDashboard.putNumber("Odo/Heading_deg", pose.getRotation().getDegrees());

        SmartDashboard.putNumber("Odo/Vx_mps", m_robotContainer.getDrivetrain().getState().Speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Odo/Vy_mps", m_robotContainer.getDrivetrain().getState().Speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Odo/Omega_radps", m_robotContainer.getDrivetrain().getState().Speeds.omegaRadiansPerSecond);
      }
    } catch (Exception e) {
      DriverStation.reportError("robotPeriodic exception: " + e.getMessage(), e.getStackTrace());
    }
  }

  @Override
  public void disabledInit() {
    // Important: teleopInit() DOES NOT rerun when you disable/re-enable mid-teleop.
    // Force arm back into MotionMagic holding so it can't get stuck "dead" on enable.
    try {
      m_robotContainer.getIntakeArmSubsystem().disableManualControl();
      m_robotContainer.getIntakeArmSubsystem().holdCurrentPosition();
    } catch (Exception e) {
      DriverStation.reportError("disabledInit exception: " + e.getMessage(), e.getStackTrace());
    }
  }

  @Override
  public void disabledPeriodic() {
    // Keep publishing while disabled (prevents Shuffleboard looking like it vanished)
    SmartDashboard.putNumber("Arm Degrees", m_robotContainer.getIntakeArmSubsystem().getDegrees());
  }

  @Override
  public void autonomousInit() {
    seedHeadingForAlliance();

    // Slow drop arm on enable
    CommandScheduler.getInstance().schedule(m_robotContainer.getEnableArmDropCommand());

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    seedHeadingForAlliance();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancelAll();
    }

    // Slow drop arm on enable (only runs when teleop starts, NOT after disable/enable)
    CommandScheduler.getInstance().schedule(m_robotContainer.getEnableArmDropCommand());
  }

  @Override
  public void teleopPeriodic() {
    // Also publish in teleop
    SmartDashboard.putNumber("Arm Degrees", m_robotContainer.getIntakeArmSubsystem().getDegrees());
  }
}