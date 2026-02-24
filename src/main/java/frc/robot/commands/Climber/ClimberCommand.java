package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.Constants.ClimberConstants;

/**
 * Command to initialize the climber: zero the encoder and apply angle limits.
 * Optionally move to a target angle (degrees). If no target is provided the command
 * will zero the climber and finish.
 */
public class ClimberCommand extends Command {

	private final ClimberSubsystem climber;
	private final Double targetAngleDeg; // nullable: if null we only zero and finish
	private final double maxAngleDeg;

	/**
	 * Zero-only command (will set current position to 0 and set max limit to 90° by default).
	 */
	public ClimberCommand(ClimberSubsystem climber) {
		this(climber, null, 90.0);
	}

	/**
	 * Constructor with optional move-to-target.
	 *
	 * @param climber the climber subsystem
	 * @param targetAngleDeg nullable target angle in degrees; if null command just zeros
	 * @param maxAngleDeg maximum allowed angle in degrees (min stays 0)
	 */
	public ClimberCommand(ClimberSubsystem climber, Double targetAngleDeg, double maxAngleDeg) {
		this.climber = climber;
		this.targetAngleDeg = targetAngleDeg;
		this.maxAngleDeg = maxAngleDeg;
		addRequirements(climber);
	}

		@Override
		public void initialize() {
		// Treat current physical position as 0° (boot-zero)
		climber.zeroClimberPositionOnBoot();
		// enforce limits
		climber.setAngleLimitsDeg(0.0, maxAngleDeg);

		// If a target was provided, ask the climber to move to it
		if (targetAngleDeg != null) {
			climber.setGoalDegrees(targetAngleDeg);
		}
	}
		@Override
		public void execute() {
			// The subsystem's periodic() will drive to the target if one was set.
		}

		@Override
		public void end(boolean interrupted) {
			// always stop motors when the command ends
			climber.stop();
		}

		@Override
		public boolean isFinished() {
			// If we were given a target, finish when at target. Otherwise we only zero and finish.
			if (targetAngleDeg == null) {
				return true;
			}
			return climber.atGoalRangeDeg(targetAngleDeg, ClimberConstants.kToleranceDeg);
		}
}
