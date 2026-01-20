package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.LumenLightsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
public class SmartDashboardSubsytem extends SubsystemBase{

    public void getPos() {
        SmartDashboard.putNumber(getName(), 0);
    }
    
}
