package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ClimberConstants;

/**
 * Climber subsystem following the project's style: two TalonFX motors (left/right) that are
 * commanded together. Supports boot-zeroing (set current position to 0°), adjustable
 * min/max angles, MotionMagic position control, manual open-loop control, and handy helpers.
 */
public class ClimberSubsystem extends SubsystemBase {

	private final TalonFX leftMotor = new TalonFX(ClimberConstants.kLeftMotorId, ClimberConstants.kCanBus);
	private final TalonFX rightMotor = new TalonFX(ClimberConstants.kRightMotorId, ClimberConstants.kCanBus);

	private final MotionMagicVoltage mm = new MotionMagicVoltage(0.0);

	private double goalMotorRot = 0.0;

	private boolean manualControl = false;
	private double manualClimberRps = 0.0;

	private boolean bootZeroDone = false;

	private double minDeg = ClimberConstants.kDefaultMinDeg;
	private double maxDeg = ClimberConstants.kDefaultMaxDeg;

	public ClimberSubsystem() {
		leftMotor.setNeutralMode(NeutralModeValue.Brake);
		rightMotor.setNeutralMode(NeutralModeValue.Brake);

		applyConfig(
				ClimberConstants.kP,
				ClimberConstants.kI,
				ClimberConstants.kD,
				ClimberConstants.kCruiseRps,
				ClimberConstants.kAccelRps2
		);

		SmartDashboard.putBoolean("Climber/BootZeroDone", false);
		SmartDashboard.putNumber("Climber/Degrees", 0.0);
	}

	private void applyConfig(double kP, double kI, double kD, double cruiseRps, double accelRps2) {
		TalonFXConfiguration cfg = new TalonFXConfiguration();

		cfg.Slot0.kP = kP;
		cfg.Slot0.kI = kI;
		cfg.Slot0.kD = kD;

		cfg.MotionMagic.MotionMagicCruiseVelocity = climberRpsToMotorRps(cruiseRps);
		cfg.MotionMagic.MotionMagicAcceleration   = climberRps2ToMotorRps2(accelRps2);

		leftMotor.getConfigurator().apply(cfg);
		rightMotor.getConfigurator().apply(cfg);
	}

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Climber/Degrees", getDegrees());

		if (manualControl) {
			leftMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(climberRpsToMotorRps(manualClimberRps)));
			rightMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(climberRpsToMotorRps(manualClimberRps)));
		} else {
			leftMotor.setControl(mm.withPosition(goalMotorRot));
			rightMotor.setControl(mm.withPosition(goalMotorRot));
		}
	}

	// Call once at boot before commands run
	public void zeroClimberPositionOnBoot() {
		if (bootZeroDone) return;
		leftMotor.setPosition(0.0);
		rightMotor.setPosition(0.0);
		bootZeroDone = true;
		SmartDashboard.putBoolean("Climber/BootZeroDone", true);
	}

	public void setAngleLimitsDeg(double min, double max) {
		if (min > max) return;
		this.minDeg = min;
		this.maxDeg = max;
		// clamp current goal
		setGoalDegrees(getDegrees());
	}

	public void enableManualClimberRPS(double climberRps) {
		manualClimberRps = climberRps;
		manualControl = true;
	}

	public void disableManualControl() {
		manualControl = false;
	}

	public void setGoalDegrees(double deg) {
		manualControl = false;
		double clamped = Math.max(minDeg, Math.min(maxDeg, deg));
		goalMotorRot = degreesToMotorRotations(clamped);
	}

	public void holdCurrentPosition() {
		manualControl = false;
		goalMotorRot = getMotorRotations();
	}

	public double getDegrees() {
		return motorRotationsToDegrees(getMotorRotations());
	}

	public boolean atGoalRangeDeg(double goalDeg, double tolDeg) {
		return Math.abs(getDegrees() - goalDeg) <= tolDeg;
	}

	public void stop() {
		leftMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(0.0));
		rightMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(0.0));
	}

	private double getMotorRotations() {
		return leftMotor.getPosition().getValueAsDouble();
	}

	private static double degreesToMotorRotations(double climberDeg) {
		double climberRot = climberDeg / 360.0;
		return climberRot * ClimberConstants.kMotorRotationsPerClimberRotation;
	}

	private static double motorRotationsToDegrees(double motorRot) {
		double climberRot = motorRot / ClimberConstants.kMotorRotationsPerClimberRotation;
		return climberRot * 360.0;
	}

	private static double climberRpsToMotorRps(double climberRps) {
		return climberRps * ClimberConstants.kMotorRotationsPerClimberRotation;
	}

	private static double climberRps2ToMotorRps2(double climberRps2) {
		return climberRps2 * ClimberConstants.kMotorRotationsPerClimberRotation;
	}
}
