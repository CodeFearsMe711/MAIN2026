package frc.robot.subsystems.LEDS;

import java.util.Optional;

import com.lumynlabs.connection.usb.USBPort;
import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.domain.led.DirectLED;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ConnectorXLeds extends SubsystemBase {
  private static final String ZONE_NAME = "TestZone";
  private static final int LED_COUNT = 300;

  private static final double kBlinkPeriodSec = 0.20;
  private static final double kPreActiveWindowSec = 5.0;

  private final ConnectorX cx = new ConnectorX();
  private DirectLED direct;
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_COUNT);

  private boolean started = false;
  private boolean connected = false;

  private double lastBlinkToggle = 0.0;
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

      direct = cx.createDirectLED(ZONE_NAME, LED_COUNT);
    } catch (Throwable t) {
      DriverStation.reportError("ConnectorX start crashed: " + t, t.getStackTrace());
    }
  }

  private boolean isHubActiveAtMatchTime(
      double matchTimeSec,
      Optional<Alliance> allianceOpt,
      boolean isAutoEnabled,
      boolean isTeleopEnabled,
      String gameData) {

    if (allianceOpt.isEmpty()) {
      return false;
    }

    if (isAutoEnabled) {
      return true;
    }

    if (!isTeleopEnabled) {
      return false;
    }

    if (gameData == null || gameData.isEmpty()) {
      return true;
    }

    boolean redInactiveFirst;
    switch (gameData.charAt(0)) {
      case 'R':
        redInactiveFirst = true;
        break;
      case 'B':
        redInactiveFirst = false;
        break;
      default:
        return true;
    }

    boolean shift1Active;
    switch (allianceOpt.get()) {
      case Red:
        shift1Active = !redInactiveFirst;
        break;
      case Blue:
        shift1Active = redInactiveFirst;
        break;
      default:
        return true;
    }

    if (matchTimeSec > 130.0) {
      return true;
    } else if (matchTimeSec > 105.0) {
      return shift1Active;
    } else if (matchTimeSec > 80.0) {
      return !shift1Active;
    } else if (matchTimeSec > 55.0) {
      return shift1Active;
    } else if (matchTimeSec > 30.0) {
      return !shift1Active;
    } else {
      return true;
    }
  }

  private boolean isHubActiveNow(Optional<Alliance> allianceOpt, String gameData) {
    return isHubActiveAtMatchTime(
        DriverStation.getMatchTime(),
        allianceOpt,
        DriverStation.isAutonomousEnabled(),
        DriverStation.isTeleopEnabled(),
        gameData);
  }

  private boolean isPreActiveNow(Optional<Alliance> allianceOpt, String gameData) {
    if (allianceOpt.isEmpty()) {
      return false;
    }

    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    boolean activeNow = isHubActiveAtMatchTime(
        DriverStation.getMatchTime(),
        allianceOpt,
        false,
        true,
        gameData);

    boolean activeInFiveSeconds = isHubActiveAtMatchTime(
        DriverStation.getMatchTime() - kPreActiveWindowSec,
        allianceOpt,
        false,
        true,
        gameData);

    return !activeNow && activeInFiveSeconds;
  }

  private void updateBlinkState() {
    double now = Timer.getFPGATimestamp();
    if (now - lastBlinkToggle >= kBlinkPeriodSec) {
      lastBlinkToggle = now;
      blinkOn = !blinkOn;
    }
  }

  private void setAll(int r, int g, int b) {
    for (int i = 0; i < LED_COUNT; i++) {
      buffer.setRGB(i, r, g, b);
    }
  }

  @Override
  public void periodic() {
    if (!started) start();
    if (!connected || direct == null) return;

    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    String gameData = DriverStation.getGameSpecificMessage();

    boolean hubActive = isHubActiveNow(allianceOpt, gameData);
    boolean preActive = isPreActiveNow(allianceOpt, gameData);

    SmartDashboard.putBoolean("MatchHub/Active", hubActive);
    SmartDashboard.putBoolean("MatchHub/PreActive", preActive);
    SmartDashboard.putNumber("MatchHub/MatchTime", DriverStation.getMatchTime());
    SmartDashboard.putString("MatchHub/GameData", gameData);
    SmartDashboard.putString(
        "MatchHub/Alliance",
        allianceOpt.isPresent() ? allianceOpt.get().name() : "Unknown");

    int teamR = 0;
    int teamG = 0;
    int teamB = 0;

    if (allianceOpt.isPresent()) {
      Alliance alliance = allianceOpt.get();
      teamR = alliance == Alliance.Red ? 255 : 0;
      teamB = alliance == Alliance.Blue ? 255 : 0;
    } else {
      teamR = 255;
      teamG = 255;
      teamB = 0;
    }

    if (hubActive || preActive) {
      updateBlinkState();
      if (blinkOn) {
        setAll(teamR, teamG, teamB);
      } else {
        setAll(0, 0, 0);
      }
    } else {
      setAll(teamR, teamG, teamB);
    }

    try {
      direct.update(buffer);
    } catch (Throwable t2) {
      DriverStation.reportError("ConnectorX update crashed: " + t2, t2.getStackTrace());
    }
  }
}