package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PhotonVisionSubsytem;

/**
 * Command that aims the drivetrain at the best AprilTag seen by PhotonVision while preserving
 * driver translation input. P gain and latency are tunable via SmartDashboard keys:
 *  - "Photon/AimP" (default 3.0)
 *  - "Photon/Latency" (default 0.05)
 */
public class AimAtAprilTagCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final PhotonVisionSubsytem m_vision;
    private final DoubleSupplier m_vxSupplier;
    private final DoubleSupplier m_vySupplier;
    private final double m_maxAngularRate;

    public AimAtAprilTagCommand(
        CommandSwerveDrivetrain drivetrain,
        PhotonVisionSubsytem vision,
        DoubleSupplier vxSupplier,
        DoubleSupplier vySupplier,
        double maxAngularRate
    ) {
        m_drivetrain = drivetrain;
        m_vision = vision;
        m_vxSupplier = vxSupplier;
        m_vySupplier = vySupplier;
        m_maxAngularRate = maxAngularRate;
        addRequirements(drivetrain);
    }
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double vx = m_vxSupplier.getAsDouble();
        double vy = m_vySupplier.getAsDouble();

        double kAimP = SmartDashboard.getNumber("Photon/AimP", 3.0);
        double latency = SmartDashboard.getNumber("Photon/Latency", 0.05);

        double rot = 0.0;
        if (m_vision.hasTarget()) {
            double yawDeg = m_vision.getAdjustedYawForMotion(vx, vy, latency);
            double errorRad = Math.toRadians(yawDeg);
            rot = MathUtil.clamp(kAimP * errorRad, -m_maxAngularRate, m_maxAngularRate);
            SmartDashboard.putNumber("Photon/PredictedYawDeg", yawDeg);
            SmartDashboard.putString("Photon/TargetStatus", "Target") ;
        } else {
            SmartDashboard.putString("Photon/TargetStatus", "NoTarget");
        }

        SwerveRequest request = new SwerveRequest.FieldCentric()
            .withVelocityX(vx)
            .withVelocityY(vy)
            .withRotationalRate(rot);

        m_drivetrain.setControl(request);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.setControl(new SwerveRequest.Idle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
