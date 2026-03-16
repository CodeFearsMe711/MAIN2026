package frc.robot.subsystems.LEDS;


import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.domain.led.DirectLED;


import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ConnectorXLeds extends SubsystemBase {
  private static final String ZONE_NAME = "TestZone";
  private static final int LED_COUNT = 300;


  private final ConnectorX cx = new ConnectorX();
  private DirectLED direct;
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_COUNT);


  private boolean started = false;
  private boolean connected = false;


  private double lastBlinkToggle = 0;
  private boolean blinkOn = false;


  private void start() {
    if (started) return;
    started = true;


    try {
      boolean ok = cx.Connect(USBPort.kUSB1);
      if (!ok) ok = cx.Connect(USBPort.kUSB2);


      connected = ok;
      DriverStation.reportWarning("ConnectorX connected=" + ok, false);
      if (!ok) return;


      // If your library has a "get/open" for an existing zone, use it.
      // If it doesn't, createDirectLED must match the zone + count.
      direct = cx.createDirectLED(ZONE_NAME, LED_COUNT);


    } catch (Throwable t) {
      DriverStation.reportError("ConnectorX start crashed: " + t, t.getStackTrace());
    }
  }


  @Override
  public void periodic() {
    if (!started) start();
    if (!connected || direct == null) return;


    // Determine hub active flag from SmartDashboard (try several common keys).
    boolean hubActive = SmartDashboard.getBoolean("Hub Active",
      SmartDashboard.getBoolean("MatchHub/Active",
        SmartDashboard.getBoolean("MatchHubActive",
          SmartDashboard.getBoolean("HubActive", false))));


    // Example behavior:
    // - If hub active: blink alliance color
    // - Else if alliance known: solid red/blue
    // - Else: blink yellow (as before)


    var allianceOpt = DriverStation.getAlliance();
    int r = 0, g = 0, b = 0;


    if (hubActive && allianceOpt.isPresent()) {
      // Blink the alliance color
      Alliance a = allianceOpt.get();
      int ar = (a == Alliance.Red) ? 255 : 0;
      int ab = (a == Alliance.Blue) ? 255 : 0;


      double t = Timer.getFPGATimestamp();
      if (t - lastBlinkToggle > 0.4) {
        lastBlinkToggle = t;
        blinkOn = !blinkOn;
      }
      if (blinkOn) {
        r = ar;
        g = 0;
        b = ab;
      } else {
        r = 0;
        g = 0;
        b = 0;
      }
    } else if (allianceOpt.isPresent()) {
      // Solid alliance color when hub not active
      Alliance a = allianceOpt.get();
      r = (a == Alliance.Red) ? 255 : 0;
      g = 0;
      b = (a == Alliance.Blue) ? 255 : 0;
    } else {
      // Alliance unknown: blink yellow
      double t = Timer.getFPGATimestamp();
      if (t - lastBlinkToggle > 0.5) {
        lastBlinkToggle = t;
        blinkOn = !blinkOn;
      }
      r = blinkOn ? 255 : 0;
      g = blinkOn ? 255 : 0;
      b = 0;
    }


    for (int i = 0; i < LED_COUNT; i++) buffer.setRGB(i, r, g, b);


    try {
      direct.update(buffer);
    } catch (Throwable t2) {
      DriverStation.reportError("ConnectorX update crashed: " + t2, t2.getStackTrace());
    }
  }
}

