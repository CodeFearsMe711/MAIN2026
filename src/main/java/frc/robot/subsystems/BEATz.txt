package frc.robot.subsystems;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeArmConstants;
import frc.robot.SWERVE.TunerConstants;

public class BEATz extends SubsystemBase {

  private final Orchestra orchestra = new Orchestra();

  public BEATz() {

    String swerveBus = TunerConstants.kCANBus.getName();

    // ===== SWERVE =====
    add(3, swerveBus);
    add(4, swerveBus);

    add(5, swerveBus);
    add(6, swerveBus);

    add(1, swerveBus);
    add(2, swerveBus);

    add(7, swerveBus);
    add(8, swerveBus);

    // ===== OTHER MOTORS =====
    add(30, "");
    add(31, "");
    add(IntakeArmConstants.kMotorId, IntakeArmConstants.kCanBus);
    add(35, "");
    add(36, "");

    orchestra.loadMusic("EnableBeatz.chrp");
    orchestra.play();
  }

  private void add(int id, String canbus) {
    TalonFX fx = new TalonFX(id, canbus);
    orchestra.addInstrument(fx);
  }

  public void stop() {
    orchestra.stop();
  }

  public void play() {
    orchestra.play();
  }
}
