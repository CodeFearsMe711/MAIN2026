package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LEDS.LumenLightsSubsystem;
public class SmartDashboardSubsytem extends SubsystemBase{

    public void getPos() {
        SmartDashboard.putNumber(getName(), 0);
    }
    
}
