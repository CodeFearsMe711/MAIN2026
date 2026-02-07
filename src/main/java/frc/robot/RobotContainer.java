// src/main/java/frc/robot/RobotContainer.java
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.Constants.OperatorConstants;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.SWERVE.CommandSwerveDrivetrain;
import frc.robot.SWERVE.Telemetry;
import frc.robot.SWERVE.TunerConstants;
import frc.robot.commands.Intake.IntakeArmCommand;
import frc.robot.subsystems.Agitator.AgitatorSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Intake.IntakeArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LEDS.ConnectorXLeds;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SmartDashboardSubsytem;
import frc.robot.subsystems.Vision.PhotonVisionSubsytem;

public class RobotContainer {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  @SuppressWarnings("unused")
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  @SuppressWarnings("unused")
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);
  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  @SuppressWarnings("unused")
  private final CommandXboxController c_operatorController =
      new CommandXboxController(OperatorConstants.cDriverControllerPort);

  // Subsystems
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IntakeArmSubsystem m_intakeArmSubsystem = new IntakeArmSubsystem();
  @SuppressWarnings("unused")
  private final IntakeArmCommand m_intakeArmCommand = new IntakeArmCommand(m_intakeArmSubsystem, 90.0);
  @SuppressWarnings("unused")
  private final ConnectorXLeds m_lumenLights = new ConnectorXLeds();
  @SuppressWarnings("unused")
  private final SmartDashboardSubsytem m_SmartDashboard = new SmartDashboardSubsytem();
  @SuppressWarnings("unused")
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final ShooterSubsystem m_shootersubsystem = new ShooterSubsystem();
  private final AgitatorSubsystem m_agitatorsubsystem = new AgitatorSubsystem();

  // Vision
  private final PhotonVisionSubsytem m_photonVision = new PhotonVisionSubsytem();

  // Vision fusion gating state
  private double m_lastVisionTimestamp = -1.0;

  // Vision fusion outlier tunables
  private static final double kMaxVisionStalenessSec = 0.250;
  private static final double kBasePosTolMeters = 0.35;
  private static final double kBaseRotTolRad = Units.degreesToRadians(12.0);
  private static final double kPosTolPerSecMeters = 3.0;
  private static final double kRotTolPerSecRad = Units.degreesToRadians(360.0);

  // Hold-to-aim PID (rotation only)
  private final PIDController m_aimPid =
      new PIDController(VisionConstants.kAimKp, VisionConstants.kAimKi, VisionConstants.kAimKd);

  public RobotContainer() {
    m_aimPid.enableContinuousInput(-Math.PI, Math.PI);
    configureBindings();
  }

  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  private void configureBindings() {
    // Default drive
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

    SmartDashboard.putData("Auto Chooser", new SendableChooser<Command>());

    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    // Intake RPS
    SmartDashboard.putNumber("Intake/TargetRPS", 10);
    m_driverController.x().whileTrue(
        Commands.runEnd(
            () -> m_intakeSubsystem.setRPS(SmartDashboard.getNumber("Intake/TargetRPS", 0)),
            () -> m_intakeSubsystem.stop(),
            m_intakeSubsystem));

    // Shooter RPS
    SmartDashboard.putNumber("Shooter/TargetRPS", 15);
    m_driverController.y().whileTrue(
        Commands.runEnd(
            () -> m_shootersubsystem.setRPS(SmartDashboard.getNumber("Shooter/TargetRPS", 0)),
            () -> m_shootersubsystem.stop(),
            m_shootersubsystem));

    // Agitator RPS
    SmartDashboard.putNumber("Agitator/TargetRPS", 15);
    m_driverController.a().whileTrue(
        Commands.runEnd(
            () -> m_agitatorsubsystem.setRPS(SmartDashboard.getNumber("Agitator/TargetRPS", 0)),
            () -> m_agitatorsubsystem.stop(),
            m_agitatorsubsystem));

    // Intake arm presets
    c_operatorController.rightBumper().onTrue(new IntakeArmCommand(m_intakeArmSubsystem, 0.0));
    c_operatorController.leftBumper().onTrue(new IntakeArmCommand(m_intakeArmSubsystem, 90.0));

    // =========================
    // AIM ASSIST
    // =========================
    m_driverController.leftBumper().whileTrue(
        drivetrain.applyRequest(() -> {
          double vx = -joystick.getLeftY() * MaxSpeed;
          double vy = -joystick.getLeftX() * MaxSpeed;

          // fallback to driver rotation if no tag
          double omega = -joystick.getRightX() * MaxAngularRate;

          var yawOpt = m_photonVision.getYawToBestTagRad(VisionConstants.kAimTagIds);
          if (yawOpt.isPresent()) {
            double yawErr = yawOpt.get(); // rad, + means tag is to the right

            if (Math.abs(yawErr) >= VisionConstants.kAimMinErrorRad) {
              // Want yawErr -> 0
              double cmd = m_aimPid.calculate(yawErr, 0.0);

              // If it turns the wrong direction, remove the '-' below.
              omega = clamp(-cmd, -VisionConstants.kAimMaxOmegaRadPerSec, VisionConstants.kAimMaxOmegaRadPerSec);
            } else {
              omega = 0.0;
            }
          } else {
            m_aimPid.reset();
          }

          return drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega);
        })
    );
  }

  /** Call this every robotPeriodic() to fuse vision with strong outlier rejection. */
  public void updateVisionFusion() {
    Pose2d currentPose = drivetrain.getState().Pose;

    var opt = m_photonVision.getEstimatedGlobalPose(currentPose);
    if (opt.isEmpty()) return;

    EstimatedRobotPose est = opt.get();
    Pose2d visionPose = est.estimatedPose.toPose2d();
    double ts = est.timestampSeconds;

    double now = Timer.getFPGATimestamp();
    if ((now - ts) > kMaxVisionStalenessSec) return;

    double dt = (m_lastVisionTimestamp < 0.0) ? 0.02 : (ts - m_lastVisionTimestamp);
    if (dt <= 0.0) dt = 0.02;

    double posTol = kBasePosTolMeters + kPosTolPerSecMeters * dt;
    double rotTol = kBaseRotTolRad + kRotTolPerSecRad * dt;

    Translation2d posDelta = visionPose.getTranslation().minus(currentPose.getTranslation());
    double posErr = posDelta.getNorm();
    double rotErr = Math.abs(visionPose.getRotation().minus(currentPose.getRotation()).getRadians());

    if (est.targetsUsed.size() <= 1) {
      posTol *= 0.75;
      rotTol *= 0.75;
    }

    if (posErr > posTol) return;
    if (rotErr > rotTol) return;

    drivetrain.addVisionMeasurement(
        visionPose,
        ts,
        m_photonVision.getEstimationStdDevs(est)
    );

    m_lastVisionTimestamp = ts;
  }


  public void publishMatchHubStatus() {
  // Default
  String status = "UNKNOWN";

  // If not enabled, or no alliance yet, keep it simple
  var allianceOpt = DriverStation.getAlliance();
  if (allianceOpt.isEmpty()) {
    SmartDashboard.putString("HUB Status", status);
    return;
  }

  // During AUTO, Transition, and Endgame: BOTH hubs are active :contentReference[oaicite:1]{index=1}
  if (DriverStation.isAutonomous()) {
    SmartDashboard.putString("HUB Status", "ACTIVE");
    return;
  }

  // Teleop match time remaining (seconds)
  double t = DriverStation.getMatchTime();
  if (!(t > 0.0)) { // handles -1 or 0 when not in a real match clock
    SmartDashboard.putString("HUB Status", status);
    return;
  }

  // Endgame: 0:30–0:00 => both active :contentReference[oaicite:2]{index=2}
  if (t <= 30.0) {
    SmartDashboard.putString("HUB Status", "ACTIVE");
    return;
  }

  // Transition Shift: 2:20–2:10 => both active :contentReference[oaicite:3]{index=3}
  if (t > 130.0) {
    SmartDashboard.putString("HUB Status", "ACTIVE");
    return;
  }

  // Game-specific data: 'R' or 'B' = alliance whose hub is inactive first (SHIFT 1) :contentReference[oaicite:4]{index=4}
  String gameData = DriverStation.getGameSpecificMessage();
  if (gameData == null || gameData.length() == 0) {
    SmartDashboard.putString("HUB Status", status);
    return;
  }

  char firstInactive = gameData.charAt(0); // 'R' or 'B'
  boolean weAreRed = (allianceOpt.get() == DriverStation.Alliance.Red);

  // Determine which SHIFT we are in by teleop time remaining :contentReference[oaicite:5]{index=5}
  // SHIFT 1: 2:10–1:45 => 130–105
  // SHIFT 2: 1:45–1:20 => 105–80
  // SHIFT 3: 1:20–0:55 => 80–55
  // SHIFT 4: 0:55–0:30 => 55–30
  int shift;
  if (t > 105.0) shift = 1;
  else if (t > 80.0) shift = 2;
  else if (t > 55.0) shift = 3;
  else shift = 4;

  // If firstInactive == our alliance color => we are INACTIVE on odd shifts (1,3) and ACTIVE on even shifts (2,4). :contentReference[oaicite:6]{index=6}
  boolean weFirstInactive = (firstInactive == (weAreRed ? 'R' : 'B'));
  boolean weInactiveThisShift = weFirstInactive ? (shift == 1 || shift == 3) : (shift == 2 || shift == 4);

  status = weInactiveThisShift ? "INACTIVE" : "ACTIVE";
  SmartDashboard.putString("HUB Status", status);
}


  public Command getAutonomousCommand() {
    final var idle = new SwerveRequest.Idle();
    return Commands.sequence(
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        drivetrain.applyRequest(() ->
            drive.withVelocityX(0.5)
                .withVelocityY(0)
                .withRotationalRate(0))
            .withTimeout(5.0),
        drivetrain.applyRequest(() -> idle));
  }
}
