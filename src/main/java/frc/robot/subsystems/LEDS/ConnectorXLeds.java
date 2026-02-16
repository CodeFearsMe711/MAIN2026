package frc.robot.subsystems.LEDS;

import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.domain.led.DirectLED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConnectorXLeds extends SubsystemBase {

  private static final String ZONE_NAME = "2654 LED's";
  private static final int LED_COUNT = 300;

  private final ConnectorX cx = new ConnectorX();
  private DirectLED direct;
  private AddressableLEDBuffer buffer;

  private boolean started = false;

  private boolean on = false;
  private double lastToggle = 0;

  public ConnectorXLeds() { }

  public void start() {
    if (started) return;
    started = true;

    try {
      boolean ok = cx.Connect(USBPort.kUSB1);
      if (!ok) ok = cx.Connect(USBPort.kUSB2);

      DriverStation.reportWarning("ConnectorX connected=" + ok, false);
      if (!ok) return;

      direct = cx.createDirectLED(ZONE_NAME, LED_COUNT);
      buffer = new AddressableLEDBuffer(LED_COUNT);

      for (int i = 0; i < LED_COUNT; i++) {
        buffer.setRGB(i, 255, 255, 255); // solid white at startup
      }
      direct.update(buffer);

    } catch (Throwable t) {
      DriverStation.reportError("ConnectorX start crashed: " + t, t.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    if (direct == null || buffer == null) return;

    double t = Timer.getFPGATimestamp();
    if (t - lastToggle > 1.0) {
      lastToggle = t;
      on = !on;

      for (int i = 0; i < LED_COUNT; i++) {
        buffer.setRGB(i, on ? 255 : 0, 0, 0); // blink red
      }

      try {
        direct.update(buffer);
      } catch (Throwable t2) {
        DriverStation.reportError("ConnectorX update crashed: " + t2, t2.getStackTrace());
      }
    }
  }
}
