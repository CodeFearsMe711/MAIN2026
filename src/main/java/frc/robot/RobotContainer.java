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

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.Constants.OperatorConstants;
import frc.robot.Constants.Constants.VisionConstants;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.SWERVE.CommandSwerveDrivetrain;
import frc.robot.SWERVE.Telemetry;
import frc.robot.SWERVE.TunerConstants;
import frc.robot.commands.AimHubTagOverride;
import frc.robot.commands.Climber.ManualClimberCommand;
import frc.robot.commands.Climber.SetClimberPositionCommand;
import frc.robot.commands.Intake.IntakeArmCommand;
import frc.robot.commands.NamedCommands.NamedAgitator;
import frc.robot.commands.NamedCommands.NamedIntake;
import frc.robot.commands.NamedCommands.NamedShooter;
import frc.robot.commands.NamedCommands.NamedShooterFeed;
import frc.robot.subsystems.Agitator.AgitatorSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Intake.IntakeArmSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LEDS.ConnectorXLeds;
import frc.robot.subsystems.Shooter.ShooterFeederSubsytem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SmartDashboardSubsytem;
import frc.robot.subsystems.Vision.PhotonVisionSubsytem;
import frc.robot.util.AimAssistMath;
import frc.robot.util.ShooterMath;

public class RobotContainer {
  private final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

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

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  @SuppressWarnings("unused")
  private final CommandXboxController c_driverController =
      new CommandXboxController(OperatorConstants.cDriverControllerPort);

  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IntakeArmSubsystem m_intakeArmSubsystem = new IntakeArmSubsystem();

  @SuppressWarnings("unused")
  private final IntakeArmCommand m_intakeArmCommand =
      new IntakeArmCommand(m_intakeArmSubsystem, 90.0);

  //@SuppressWarnings("unused")
  //private final ConnectorXLeds m_lumenLights = new ConnectorXLeds();

  @SuppressWarnings("unused")
  private final SmartDashboardSubsytem m_SmartDashboard = new SmartDashboardSubsytem();

  @SuppressWarnings("unused")
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

  private final ShooterSubsystem m_shootersubsystem = new ShooterSubsystem();
  private final AgitatorSubsystem m_agitatorsubsystem = new AgitatorSubsystem();
  private final ShooterFeederSubsytem m_shooterFeederSubsytem = new ShooterFeederSubsytem();

  private final PhotonVisionSubsytem m_photonVision = new PhotonVisionSubsytem();

  // @SuppressWarnings("unused")
  // private final org.photonvision.PhotonCamera m_aimCam =
  //     new org.photonvision.PhotonCamera(VisionConstants.kCameraName);

  private final edu.wpi.first.math.controller.PIDController m_aimPid =
      new edu.wpi.first.math.controller.PIDController(6.0, 0.0, 0.0);

  @SuppressWarnings("unused")
  private final edu.wpi.first.math.controller.PIDController m_rangePid =
      new edu.wpi.first.math.controller.PIDController(
          VisionConstants.kAimRangeKp,
          VisionConstants.kAimRangeKi,
          VisionConstants.kAimRangeKd);

  private double m_lastVisionTimestamp = -1.0;

  private static final double kMaxVisionStalenessSec = 0.250;
  private static final double kBasePosTolMeters = 0.35;
  private static final double kBaseRotTolRad = Units.degreesToRadians(12.0);
  private static final double kPosTolPerSecMeters = 3.0;
  private static final double kRotTolPerSecRad = Units.degreesToRadians(360.0);

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

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  public void scheduleTeleopArmDrop() {
    getEnableArmDropCommand().schedule();
  }

  private void configurePathPlanner() {
    try {
      m_robotConfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      DriverStation.reportError(
          "RobotConfig.fromGUISettings() failed: " + e.getMessage(),
          e.getStackTrace());
      m_robotConfig = null;
    }

    if (m_robotConfig == null) {
      return;
    }

    AutoBuilder.configure(
        () -> drivetrain.getState().Pose,
        drivetrain::resetPose,
        drivetrain::getRobotRelativeSpeeds,
        drivetrain::driveRobotRelative,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)),
        m_robotConfig,
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red,
        drivetrain);
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand(
    "Shooter system",
    new NamedShooter(m_shootersubsystem, m_photonVision));
    NamedCommands.registerCommand("Shooter feed", new NamedShooterFeed(m_shooterFeederSubsytem));
    NamedCommands.registerCommand("agitater", new NamedAgitator(m_agitatorsubsystem));
    NamedCommands.registerCommand("intake", new NamedIntake(m_intakeSubsystem));

    NamedCommands.registerCommand(
        "Aim Hub Tag Override",
        new AimHubTagOverride(
            drivetrain,
            m_photonVision,
            m_shootersubsystem,
            m_aimPid));

    NamedCommands.registerCommand(
        "IntakeArmUp",
        new IntakeArmCommand(
            m_intakeArmSubsystem,
            SmartDashboard.getNumber("IntakeArm/UpDeg", IntakeArmConstants.kPosDegB)));

    NamedCommands.registerCommand(
        "IntakeArmDown",
        new IntakeArmCommand(
            m_intakeArmSubsystem,
            SmartDashboard.getNumber("IntakeArm/DownDeg", IntakeArmConstants.kPosDegA)));

    NamedCommands.registerCommand(
        "ClimberUp",
        new SetClimberPositionCommand(m_ClimberSubsystem, 1500));

    NamedCommands.registerCommand(
        "ClimberDown",
        new SetClimberPositionCommand(m_ClimberSubsystem, 1100));
  }

  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  public IntakeArmSubsystem getIntakeArmSubsystem() {
    return m_intakeArmSubsystem;
  }

  public void forceArmDownNow() {
    double downDeg =
        SmartDashboard.getNumber("IntakeArm/DownDeg", IntakeArmConstants.kPosDegA);

    double enableCruise =
        SmartDashboard.getNumber(
            "IntakeArm/EnableCruiseRps",
            IntakeArmConstants.kEnableCruiseRps_Arm);

    double enableAccel =
        SmartDashboard.getNumber(
            "IntakeArm/EnableAccelRps2",
            IntakeArmConstants.kEnableAccelRps2_Arm);

    m_intakeArmSubsystem.disableManualControl();
    m_intakeArmSubsystem.setMotionMagicConstraintsArm(enableCruise, enableAccel);
    m_intakeArmSubsystem.setGoalDegrees(downDeg);
  }

  private void configureBindings() {
    SmartDashboard.putString("ZZZ_ROBOTCONTAINER_VERSION", "ROBOTCONTAINER_NEW_BUILD_123");
    m_driverController.povDown().onTrue(
        Commands.runOnce(
            () -> drivetrain.resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0))),
            drivetrain));

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () -> {
              double ly =
                  edu.wpi.first.math.MathUtil.applyDeadband(m_driverController.getLeftY(), 0.08);
              double lx =
                  edu.wpi.first.math.MathUtil.applyDeadband(m_driverController.getLeftX(), 0.08);
              double rx =
                  edu.wpi.first.math.MathUtil.applyDeadband(m_driverController.getRightX(), 0.08);

              return drive
                  .withVelocityX(-ly * MaxSpeed)
                  .withVelocityY(-lx * MaxSpeed)
                  .withRotationalRate(-rx * MaxAngularRate);
            }));

    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
        drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    m_driverController.back().and(m_driverController.leftBumper())
        .onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);

    SmartDashboard.putNumber("Intake/TargetRPS", 40);
    c_driverController.rightBumper().whileTrue(
        Commands.runEnd(
            () -> m_intakeSubsystem.setRPS(SmartDashboard.getNumber("Intake/TargetRPS", 0)),
            () -> m_intakeSubsystem.stop(),
            m_intakeSubsystem));

    SmartDashboard.putNumber("ShooterLowPresetRPS", 75.0);
    SmartDashboard.putNumber("ShooterFastPresetRPS", 100.0);

    m_driverController.a().whileTrue(
        Commands.runEnd(
            () -> m_shootersubsystem.setManualRPS(SmartDashboard.getNumber("ShooterLowPresetRPS", 75.0)),
            () -> m_shootersubsystem.clearManualRPS()));

    m_driverController.y().whileTrue(
        Commands.runEnd(
            () -> m_shootersubsystem.setManualRPS(SmartDashboard.getNumber("ShooterFastPresetRPS",100.0)),
            () -> m_shootersubsystem.clearManualRPS()));

    SmartDashboard.putNumber("Shooter/FeedRPS", 35);
    m_driverController.rightBumper().whileTrue(
        Commands.runEnd(
            () -> m_shooterFeederSubsytem.setRPS(SmartDashboard.getNumber("Shooter/FeedRPS", 0)),
            () -> m_shooterFeederSubsytem.stop(),
            m_shooterFeederSubsytem));

    SmartDashboard.putNumber("Agitator/TargetRPS", 20);
    c_driverController.a().whileTrue(
        Commands.runEnd(
            () -> m_agitatorsubsystem.setRPS(SmartDashboard.getNumber("Agitator/TargetRPS", 0)),
            () -> m_agitatorsubsystem.stop(),
            m_agitatorsubsystem));

    SmartDashboard.putNumber("Negative Agitator/TargetRPS", -20);
    c_driverController.b().whileTrue(
        Commands.runEnd(
            () -> m_agitatorsubsystem.setRPS(
                SmartDashboard.getNumber("Negative Agitator/TargetRPS", 0)),
            () -> m_agitatorsubsystem.stop(),
            m_agitatorsubsystem));

    SmartDashboard.putNumber("IntakeArm/DownDeg", IntakeArmConstants.kPosDegA);
    SmartDashboard.putNumber("IntakeArm/UpDeg", IntakeArmConstants.kPosDegB);
    SmartDashboard.putNumber("IntakeArm/TeleopCruiseRps", IntakeArmConstants.kCruiseRps_Arm);
    SmartDashboard.putNumber("IntakeArm/TeleopAccelRps2", IntakeArmConstants.kAccelRps2_Arm);
    SmartDashboard.putNumber("IntakeArm/EnableCruiseRps", IntakeArmConstants.kEnableCruiseRps_Arm);
    SmartDashboard.putNumber("IntakeArm/EnableAccelRps2", IntakeArmConstants.kEnableAccelRps2_Arm);

    m_intakeArmSubsystem.setDefaultCommand(
        Commands.run(
            () -> {
              if (!DriverStation.isTeleopEnabled()) {
                m_intakeArmSubsystem.holdCurrentPosition();
                return;
              }

              double downDeg = SmartDashboard.getNumber("IntakeArm/DownDeg", IntakeArmConstants.kPosDegA);
              double upDeg = SmartDashboard.getNumber("IntakeArm/UpDeg", IntakeArmConstants.kPosDegB);

              double t = c_driverController.getLeftTriggerAxis();

              if (t < 0.05) {
                m_intakeArmSubsystem.setGoalDegrees(downDeg);
                return;
              }

              if (t > 1.0) {
                t = 1.0;
              }

              double targetDeg = downDeg + (upDeg - downDeg) * t;
              m_intakeArmSubsystem.setGoalDegrees(targetDeg);
            },
            m_intakeArmSubsystem));

    c_driverController.y().whileTrue(
        new ManualClimberCommand(
            m_ClimberSubsystem,
            ManualClimberCommand.ClimberDirection.UP));

    c_driverController.x().whileTrue(
        new ManualClimberCommand(
            m_ClimberSubsystem,
            ManualClimberCommand.ClimberDirection.DOWN));

   // =========================
   // AIM ASSIST + AUTO SHOOT SPEED
   // =========================
m_driverController.leftBumper().whileTrue(
    drivetrain.applyRequest(
        () -> {
          m_shootersubsystem.setVisionEnabled(true);
          SmartDashboard.putBoolean("AutoAimLBActive", true);

          double ly =
              edu.wpi.first.math.MathUtil.applyDeadband(
                  m_driverController.getLeftY(), 0.08);
          double lx =
              edu.wpi.first.math.MathUtil.applyDeadband(
                  m_driverController.getLeftX(), 0.08);

          double vx = -ly * MaxSpeed;
          double vy = -lx * MaxSpeed;
          double omega = 0.0;

          var result = m_photonVision.getLatestResult();

          if (!result.hasTargets()) {
            SmartDashboard.putBoolean("AutoAimHasAllowedTarget", false);
            return drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega);
          }

          // TARGET USED FOR AIMING
          var bestAimTarget =
              AimAssistMath.findBestAllowedTarget(
                  result.getTargets(),
                  VisionConstants.kAimTagIds);

          // TARGET USED FOR SHOOTER SPEED
          var bestShooterTarget =
              AimAssistMath.findBestAllowedTarget(
                  result.getTargets(),
                  VisionConstants.kShooterTagIds);

          if (bestAimTarget == null) {
            SmartDashboard.putBoolean("AutoAimHasAllowedTarget", false);
            return drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega);
          }

          SmartDashboard.putBoolean("AutoAimHasAllowedTarget", true);

          // SHOOTER SPEED FROM SHOOTER TAG IDS
          if (bestShooterTarget != null) {
            double area = bestShooterTarget.getArea();
            double shooterRps = ShooterMath.tagAreaToShooterRps(area);

            SmartDashboard.putNumber("AutoAimArea", area);
            SmartDashboard.putNumber("AutoAimShooterTargetRPS", shooterRps);

            m_shootersubsystem.updateVisionSpeed(shooterRps);
          }

          // AIMING FROM AIM TAG IDS
          var robotSpeeds = drivetrain.getRobotRelativeSpeeds();
          var yawOpt = AimAssistMath.getCorrectedYawRad(bestAimTarget, robotSpeeds);

          if (yawOpt.isPresent()) {
            double yawErrRad = yawOpt.get();

            SmartDashboard.putNumber(
                "AutoAimYawErrDeg",
                edu.wpi.first.math.util.Units.radiansToDegrees(yawErrRad));

            if (Math.abs(yawErrRad) < VisionConstants.kAimMinErrorRad) {
              omega = 0.0;
              m_aimPid.reset();
            } else {
              double cmd = m_aimPid.calculate(yawErrRad, 0.0);

              omega =
                  clamp(
                      cmd,
                      -VisionConstants.kAimMaxOmegaRadPerSec,
                      VisionConstants.kAimMaxOmegaRadPerSec);
            }

            SmartDashboard.putNumber("AutoAimOmegaCmd", omega);
          } else {
            m_aimPid.reset();
          }

          return drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega);
        }));

m_driverController.leftBumper().onFalse(
    Commands.runOnce(
        () -> {
          SmartDashboard.putBoolean("AutoAimLBActive", false);
          SmartDashboard.putBoolean("AutoAimHasAllowedTarget", false);
          m_aimPid.reset();
          m_shootersubsystem.setVisionEnabled(false);
        }));
          }
  public void updateVisionFusion() {
    Pose2d currentPose = drivetrain.getState().Pose;

    var opt = m_photonVision.getEstimatedGlobalPose(currentPose);
    if (opt.isEmpty()) {
      return;
    }

    EstimatedRobotPose est = opt.get();
    Pose2d visionPose = est.estimatedPose.toPose2d();
    double ts = est.timestampSeconds;

    double now = Timer.getFPGATimestamp();
    if ((now - ts) > kMaxVisionStalenessSec) {
      return;
    }

    double dt = (m_lastVisionTimestamp < 0.0) ? 0.02 : (ts - m_lastVisionTimestamp);
    if (dt <= 0.0) {
      dt = 0.02;
    }

    double posTol = kBasePosTolMeters + kPosTolPerSecMeters * dt;
    double rotTol = kBaseRotTolRad + kRotTolPerSecRad * dt;

    Translation2d posDelta =
        visionPose.getTranslation().minus(currentPose.getTranslation());
    double posErr = posDelta.getNorm();
    double rotErr =
        Math.abs(visionPose.getRotation().minus(currentPose.getRotation()).getRadians());

    if (est.targetsUsed.size() <= 1) {
      posTol *= 0.75;
      rotTol *= 0.75;
    }

    if (posErr > posTol) {
      return;
    }

    if (rotErr > rotTol) {
      return;
    }

    drivetrain.addVisionMeasurement(
        visionPose,
        ts,
        m_photonVision.getEstimationStdDevs(est));

    m_lastVisionTimestamp = ts;
  }

  public void publishMatchHubStatus() {
  }


  public void logAdvantageKit() {
    var driveState = drivetrain.getState();
    Logger.recordOutput("Drive/Pose", driveState.Pose);
    Logger.recordOutput("Drive/ModuleStates", driveState.ModuleStates);
    Logger.recordOutput("Drive/ModuleTargets", driveState.ModuleTargets);
    Logger.recordOutput("Drive/ModulePositions", driveState.ModulePositions);
    Logger.recordOutput("Drive/OdometryPeriodSec", driveState.OdometryPeriod);

    Logger.recordOutput("IntakeArm/Degrees", m_intakeArmSubsystem.getDegrees());

    Logger.recordOutput("Climber/Degrees", m_ClimberSubsystem.getDegrees());
    Logger.recordOutput("Climber/LeftDegrees", m_ClimberSubsystem.getLeftDegrees());
    Logger.recordOutput("Climber/RightDegrees", m_ClimberSubsystem.getRightDegrees());
    Logger.recordOutput("Climber/LeftBottomPressed", m_ClimberSubsystem.isLeftBottomPressed());
    Logger.recordOutput("Climber/RightBottomPressed", m_ClimberSubsystem.isRightBottomPressed());
    Logger.recordOutput("Climber/LeftBottomVoltage", m_ClimberSubsystem.getLeftBottomVoltage());
    Logger.recordOutput("Climber/RightBottomVoltage", m_ClimberSubsystem.getRightBottomVoltage());

    Logger.recordOutput("Shooter/MotorRPS", m_shootersubsystem.getMotorRPS());

    var visionResult = m_photonVision.getLatestResult();
    Logger.recordOutput("Vision/HasTargets", visionResult.hasTargets());

    if (visionResult.hasTargets()) {
      var bestTarget = visionResult.getBestTarget();
      Logger.recordOutput("Vision/BestTarget/Id", bestTarget.getFiducialId());
      Logger.recordOutput("Vision/BestTarget/YawDeg", bestTarget.getYaw());
      Logger.recordOutput("Vision/BestTarget/PitchDeg", bestTarget.getPitch());
      Logger.recordOutput("Vision/BestTarget/Area", bestTarget.getArea());
    } else {
      Logger.recordOutput("Vision/BestTarget/Id", -1);
      Logger.recordOutput("Vision/BestTarget/YawDeg", 0.0);
      Logger.recordOutput("Vision/BestTarget/PitchDeg", 0.0);
      Logger.recordOutput("Vision/BestTarget/Area", 0.0);
    }
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public Command getEnableArmDropCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              double enableCruise =
                  SmartDashboard.getNumber(
                      "IntakeArm/EnableCruiseRps",
                      IntakeArmConstants.kEnableCruiseRps_Arm);

              double enableAccel =
                  SmartDashboard.getNumber(
                      "IntakeArm/EnableAccelRps2",
                      IntakeArmConstants.kEnableAccelRps2_Arm);

              m_intakeArmSubsystem.disableManualControl();
              m_intakeArmSubsystem.setMotionMagicConstraintsArm(enableCruise, enableAccel);

              double downDeg =
                  SmartDashboard.getNumber(
                      "IntakeArm/DownDeg",
                      IntakeArmConstants.kPosDegA);

              m_intakeArmSubsystem.setGoalDegrees(downDeg);
            },
            m_intakeArmSubsystem),
        Commands.waitUntil(
                () -> {
                  double downDeg =
                      SmartDashboard.getNumber(
                          "IntakeArm/DownDeg",
                          IntakeArmConstants.kPosDegA);

                  double tolDeg =
                      SmartDashboard.getNumber(
                          "IntakeArm/CompleteTolDeg",
                          4.0);

                  return m_intakeArmSubsystem.atGoalRangeDeg(downDeg, tolDeg);
                })
            .withTimeout(1.75),
        Commands.runOnce(
            () -> {
              double teleopCruise =
                  SmartDashboard.getNumber(
                      "IntakeArm/TeleopCruiseRps",
                      IntakeArmConstants.kCruiseRps_Arm);

              double teleopAccel =
                  SmartDashboard.getNumber(
                      "IntakeArm/TeleopAccelRps2",
                      IntakeArmConstants.kAccelRps2_Arm);

              m_intakeArmSubsystem.setMotionMagicConstraintsArm(teleopCruise, teleopAccel);
            },
            m_intakeArmSubsystem));
  }
}