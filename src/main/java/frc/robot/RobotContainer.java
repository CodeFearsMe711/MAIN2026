// src/main/java/frc/robot/RobotContainer.java
package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
import frc.robot.Constants.IntakeArmConstants;

import frc.robot.SWERVE.CommandSwerveDrivetrain;
import frc.robot.SWERVE.Telemetry;
import frc.robot.SWERVE.TunerConstants;

import frc.robot.commands.Intake.IntakeArmCommand;
import frc.robot.commands.Intake.IntakeArmEnableDropCommand;
import frc.robot.commands.AimHubTagOverride;

import frc.robot.subsystems.Agitator.AgitatorSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Intake.IntakeArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LEDS.ConnectorXLeds;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SmartDashboardSubsytem;
import frc.robot.subsystems.Shooter.ShooterFeederSubsytem;
import frc.robot.subsystems.Vision.PhotonVisionSubsytem;

import frc.robot.commands.NamedCommands.*;

public class RobotContainer {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1)
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
  private final ShooterFeederSubsytem m_shooterFeederSubsytem = new ShooterFeederSubsytem();

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

  // PathPlanner
  private RobotConfig m_robotConfig;
  private final SendableChooser<Command> m_autoChooser;

  public RobotContainer() {
    configureNamedCommands();

    m_aimPid.enableContinuousInput(-Math.PI, Math.PI);

    configurePathPlanner();

    m_autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    configureBindings();
  }

  private void configurePathPlanner() {
    try {
      m_robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError("RobotConfig.fromGUISettings() failed: " + e.getMessage(), e.getStackTrace());
      m_robotConfig = null;
    }

    if (m_robotConfig == null) return;

    AutoBuilder.configure(
        () -> drivetrain.getState().Pose,
        (Pose2d pose) -> {
          // NOTE:
          // Your CTRE drivetrain file doesn't expose a Pose2d reset method.
          // This keeps PathPlanner compiling and at least seeds the heading.
          // If you add a real resetPose(Pose2d) method later, replace this.
          drivetrain.seedFieldCentric(pose.getRotation());
        },
        () -> drivetrain.getState().Speeds, // ChassisSpeeds (robot-relative) from CTRE state
        (ChassisSpeeds speeds) -> {
          // Robot-relative drive for PathPlanner
          final var robotCentric =
              new SwerveRequest.RobotCentric()
                  .withDriveRequestType(DriveRequestType.Velocity)
                  .withVelocityX(speeds.vxMetersPerSecond)
                  .withVelocityY(speeds.vyMetersPerSecond)
                  .withRotationalRate(speeds.omegaRadiansPerSecond);

          drivetrain.setControl(robotCentric);
        },
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // translation
            new PIDConstants(5.0, 0.0, 0.0)  // rotation
        ),
        m_robotConfig,
        () -> DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        drivetrain
    );
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Shooter system", new NamedShooter(m_shootersubsystem));
    NamedCommands.registerCommand("Shooter feed", new NamedShooterFeed(m_shooterFeederSubsytem));
    NamedCommands.registerCommand("agitater", new NamedAgitator(m_agitatorsubsystem));
    NamedCommands.registerCommand("intake arm", new NamedIntakeArm(m_intakeArmSubsystem));
    NamedCommands.registerCommand("intake", new NamedIntake(m_intakeSubsystem));
    NamedCommands.registerCommand("Aim Hub Tag Override", new AimHubTagOverride(drivetrain, m_photonVision, m_aimPid));
  }

  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  private void configureBindings() {
    // Default drive
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                      .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                      .withRotationalRate(-joystick.getRightX() * MaxAngularRate)));

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
    SmartDashboard.putNumber("Intake/TargetRPS", 40);
    m_driverController.x().whileTrue(
        Commands.runEnd(
            () -> m_intakeSubsystem.setRPS(SmartDashboard.getNumber("Intake/TargetRPS", 0)),
            () -> m_intakeSubsystem.stop(),
            m_intakeSubsystem));

    // Shooter RPS
    SmartDashboard.putNumber("Shooter/TargetRPS", 160);
    m_driverController.y().whileTrue(
        Commands.runEnd(
            () -> m_shootersubsystem.setRPS(SmartDashboard.getNumber("Shooter/TargetRPS", 0)),
            () -> m_shootersubsystem.stop(),
            m_shootersubsystem));

    // Shooter feeder RPS
    SmartDashboard.putNumber("Shooter/FeedRPS", 35);
    m_driverController.rightBumper().whileTrue(
        Commands.runEnd(
            () -> m_shooterFeederSubsytem.setRPS(SmartDashboard.getNumber("Shooter/FeedRPS", 0)),
            () -> m_shooterFeederSubsytem.stop(),
            m_shooterFeederSubsytem));

    // Agitator RPS
    SmartDashboard.putNumber("Agitator/TargetRPS", 35);
    m_driverController.rightBumper().whileTrue(
        Commands.runEnd(
            () -> m_agitatorsubsystem.setRPS(SmartDashboard.getNumber("Agitator/TargetRPS", 0)),
            () -> m_agitatorsubsystem.stop(),
            m_agitatorsubsystem));

    // =========================
    // INTAKE ARM (Trigger position)
    // =========================
    SmartDashboard.putNumber("IntakeArm/DownDeg", IntakeArmConstants.kPosDegA);
    SmartDashboard.putNumber("IntakeArm/UpDeg", IntakeArmConstants.kPosDegB);

    SmartDashboard.putNumber("IntakeArm/TeleopCruiseRps", IntakeArmConstants.kCruiseRps_Arm);
    SmartDashboard.putNumber("IntakeArm/TeleopAccelRps2", IntakeArmConstants.kAccelRps2_Arm);

    SmartDashboard.putNumber("IntakeArm/EnableCruiseRps", IntakeArmConstants.kEnableCruiseRps_Arm);
    SmartDashboard.putNumber("IntakeArm/EnableAccelRps2", IntakeArmConstants.kEnableAccelRps2_Arm);

    m_intakeArmSubsystem.setDefaultCommand(
        Commands.run(
            () -> {
              double downDeg = SmartDashboard.getNumber("IntakeArm/DownDeg", IntakeArmConstants.kPosDegA);
              double upDeg   = SmartDashboard.getNumber("IntakeArm/UpDeg", IntakeArmConstants.kPosDegB);

              double t = m_driverController.getLeftTriggerAxis();
              if (t < 0.05) t = 0.0;
              if (t > 1.0) t = 1.0;

              double targetDeg = downDeg + (upDeg - downDeg) * t;
              m_intakeArmSubsystem.setGoalDegrees(targetDeg);
            },
            m_intakeArmSubsystem
        )
    );

    // =========================
    // AIM ASSIST (teleop)
    // =========================
    m_driverController.leftBumper().whileTrue(
        drivetrain.applyRequest(() -> {
          double vx = -joystick.getLeftY() * MaxSpeed;
          double vy = -joystick.getLeftX() * MaxSpeed;

          double omega = -joystick.getRightX() * MaxAngularRate;

          var yawOpt = m_photonVision.getYawToBestTagRad(VisionConstants.kAimTagIds);
          if (yawOpt.isPresent()) {
            double yawErr = yawOpt.get();

            if (Math.abs(yawErr) >= VisionConstants.kAimMinErrorRad) {
              double cmd = m_aimPid.calculate(yawErr, 0.0);
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

  // Schedule this on enable in Robot.java
  public Command getEnableArmDropCommand() {
    return new IntakeArmEnableDropCommand(m_intakeArmSubsystem);
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
    String status = "UNKNOWN";

    var allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) {
      SmartDashboard.putString("HUB Status", status);
      return;
    }

    if (DriverStation.isAutonomous()) {
      SmartDashboard.putString("HUB Status", "ACTIVE");
      return;
    }

    double t = DriverStation.getMatchTime();
    if (!(t > 0.0)) {
      SmartDashboard.putString("HUB Status", status);
      return;
    }

    if (t <= 30.0) {
      SmartDashboard.putString("HUB Status", "ACTIVE");
      return;
    }

    if (t > 130.0) {
      SmartDashboard.putString("HUB Status", "ACTIVE");
      return;
    }

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.length() == 0) {
      SmartDashboard.putString("HUB Status", status);
      return;
    }

    char firstInactive = gameData.charAt(0);
    boolean weAreRed = (allianceOpt.get() == DriverStation.Alliance.Red);

    int shift;
    if (t > 105.0) shift = 1;
    else if (t > 80.0) shift = 2;
    else if (t > 55.0) shift = 3;
    else shift = 4;

    boolean weFirstInactive = (firstInactive == (weAreRed ? 'R' : 'B'));
    boolean weInactiveThisShift = weFirstInactive ? (shift == 1 || shift == 3) : (shift == 2 || shift == 4);

    status = weInactiveThisShift ? "INACTIVE" : "ACTIVE";
    SmartDashboard.putString("HUB Status", status);
  }

  public Command getAutonomousCommand() {
    // PathPlanner auto selected from dashboard
    return m_autoChooser.getSelected();
  }
 
}
