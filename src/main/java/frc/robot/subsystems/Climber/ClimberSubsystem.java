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

	// Motor direction/sign correction: motor ID 37 (left) is mounted so its
	// positive output is the opposite of the physical climber rotation.
	// Use this sign when commanding the left motor and when reading its encoder.
	// Flipped from -1.0 to 1.0 to invert the climber direction per user request.
	private static final double kLeftMotorSign = 1.0;

	// Sign correction for the right motor. If the right side runs opposite the
	// expected physical direction, set this to -1.0. (User reported right side
	// moving opposite, so it's initialized to -1.0.)
	private static final double kRightMotorSign = -1.0;
	

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

		// Publish stator current for both climber motors so we can monitor each side
		double leftStatorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
		double rightStatorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();
		SmartDashboard.putNumber("Climber/Left Stator Current", leftStatorCurrent);
		SmartDashboard.putNumber("Climber/Right Stator Current", rightStatorCurrent);

		if (manualControl) {
			// Apply sign correction to the left motor so positive RPS moves the climber
			// in the same physical direction as the right motor.
			leftMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(kLeftMotorSign * climberRpsToMotorRps(manualClimberRps)));
			rightMotor.setControl(new com.ctre.phoenix6.controls.VelocityVoltage(kRightMotorSign * climberRpsToMotorRps(manualClimberRps)));
		} else {
			// For position control we must also invert the left motor command so both
			// sides move the climber to the same physical angle.
			leftMotor.setControl(mm.withPosition(kLeftMotorSign * goalMotorRot));
			rightMotor.setControl(mm.withPosition(kRightMotorSign * goalMotorRot));
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
		// Turn off manual control and hold the current position so the climber
		// doesn't drive to a previously-set goal when a button is released.
		holdCurrentPosition();
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
		// Use the average of both (sign-corrected) encoder readings so we have a
		// single, robust climber-rotation position. This avoids returning to 0
		// if one encoder is zeroed or reads incorrectly.
		double left = kLeftMotorSign * leftMotor.getPosition().getValueAsDouble();
		double right = kRightMotorSign * rightMotor.getPosition().getValueAsDouble();
		return 0.5 * (left + right);
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
