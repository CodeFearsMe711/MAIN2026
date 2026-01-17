package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Simple lights subsystem that sets the entire LED strip to the alliance color.
 * Uses WPILib AddressableLED so it will work even if Lumyn isn't available; you
 * can replace the implementation with Lumyn calls if desired.
 */
public class LumenLightsSubsystem extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_buffer;
  // Track last seen alliance; null means we haven't set anything yet / currently no alliance
  private Alliance m_currentAlliance = null;

  public LumenLightsSubsystem() {
    m_led = new AddressableLED(Constants.Lights.kLedPort);
    m_buffer = new AddressableLEDBuffer(Constants.Lights.kLedLength);
    m_led.setLength(m_buffer.getLength());
    // Start with the LEDs off
    setAllRGB(0, 0, 0);
    m_led.start();
  }

  @Override
  public void periodic() {
    // Check alliance each loop and update color when it changes. DriverStation.getAlliance()
    // returns an Optional<Alliance> (empty if unknown/not connected).
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isPresent()) {
      Alliance alliance = allianceOpt.get();
      if (alliance != m_currentAlliance) {
        m_currentAlliance = alliance;
        if (alliance == Alliance.Red) {
          setAllRGB(255, 0, 0);
        } else {
          setAllRGB(0, 0, 255);
        }
      }
    } else {
      // No alliance info (e.g., disabled before match); turn off if previously set
      if (m_currentAlliance != null) {
        m_currentAlliance = null;
        setAllRGB(0, 0, 0);
      }
    }
  }

  /** Set every LED in the buffer to the given RGB color (0-255). */
  public void setAllRGB(int r, int g, int b) {
    for (int i = 0; i < m_buffer.getLength(); i++) {
      m_buffer.setRGB(i, r, g, b);
    }
    m_led.setData(m_buffer);
  }

  /** Stop the LED output (useful for disabling). */
  public void stop() {
    m_led.stop();
  }
}
