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
  private static final String kHubStartModeKey = "MatchHubStartMode";
  private static final String kAllianceOverrideKey = "MatchHubAllianceOverride";
  private static final String ZONE_NAME = "TestZone";
  private static final int LED_COUNT = 300;

  private static final double kBlinkPeriodSec = 0.20;
  private static final double kFastBlinkPeriodSec = 0.08;
  private static final double kPreActiveWindowSec = 5.0;
  private static final double kFastPreActiveWindowSec = 2.0;

  private final ConnectorX cx = new ConnectorX();
  private DirectLED direct;
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(LED_COUNT);

  private boolean started = false;
  private boolean connected = false;
  private boolean dashboardDefaultsPublished = false;

  private boolean wasFmsAttached = false;
  private boolean wasTeleopEnabled = false;
  private double fmsAttachTimestampSec = Double.NaN;
  private double teleopStartTimestampSec = Double.NaN;

  private static final double kFirstTeleopWindowSec = 15.0;
  private static final double kSwapTeleopWindowSec = 25.0;
  private static final double kFinalActiveWindowStartSec =
      kFirstTeleopWindowSec + (3.0 * kSwapTeleopWindowSec);

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

  private void publishDashboardDefaults() {
    if (dashboardDefaultsPublished) {
      return;
    }

    SmartDashboard.putString(kHubStartModeKey, "AUTO");
    SmartDashboard.putString(kAllianceOverrideKey, "AUTO");
    dashboardDefaultsPublished = true;
  }

  private Optional<Alliance> getEffectiveAlliance(Optional<Alliance> allianceOpt) {
    if (allianceOpt.isPresent()) {
      return allianceOpt;
    }

    String override = SmartDashboard.getString(kAllianceOverrideKey, "AUTO").trim().toUpperCase();
    switch (override) {
      case "RED":
        return Optional.of(Alliance.Red);
      case "BLUE":
        return Optional.of(Alliance.Blue);
      default:
        return Optional.empty();
    }
  }

  private boolean isRedActiveFirst(String gameData) {
    if (gameData != null && !gameData.isEmpty()) {
      switch (Character.toUpperCase(gameData.charAt(0))) {
        case 'R':
          return true;
        case 'B':
          return false;
        default:
          break;
      }
    }

    String startMode = SmartDashboard.getString(kHubStartModeKey, "AUTO").trim().toUpperCase();
    switch (startMode) {
      case "RED":
        return true;
      case "BLUE":
        return false;
      default:
        return true;
    }
  }

  private boolean isAllianceActiveInSegment(Alliance alliance, boolean redActive) {
    return (alliance == Alliance.Red && redActive)
        || (alliance == Alliance.Blue && !redActive);
  }

  private boolean isHubActiveAtTeleopElapsed(
      double teleopElapsedSec,
      Optional<Alliance> allianceOpt,
      String gameData) {
    if (allianceOpt.isEmpty()) {
      return false;
    }

    boolean redActiveFirst = isRedActiveFirst(gameData);
    Alliance alliance = allianceOpt.get();

    if (teleopElapsedSec < kFirstTeleopWindowSec) {
      return isAllianceActiveInSegment(alliance, redActiveFirst);
    }

    if (teleopElapsedSec < kFirstTeleopWindowSec + kSwapTeleopWindowSec) {
      return isAllianceActiveInSegment(alliance, !redActiveFirst);
    }

    if (teleopElapsedSec < kFirstTeleopWindowSec + (2.0 * kSwapTeleopWindowSec)) {
      return isAllianceActiveInSegment(alliance, redActiveFirst);
    }

    if (teleopElapsedSec < kFinalActiveWindowStartSec) {
      return isAllianceActiveInSegment(alliance, !redActiveFirst);
    }

    return true;
  }

  private boolean isHubActiveNow(
      double teleopElapsedSec,
      Optional<Alliance> allianceOpt,
      String gameData) {
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }

    if (!DriverStation.isTeleopEnabled() || Double.isNaN(teleopElapsedSec)) {
      return false;
    }

    return isHubActiveAtTeleopElapsed(teleopElapsedSec, allianceOpt, gameData);
  }

  private boolean willBeActiveInSeconds(
      double teleopElapsedSec,
      Optional<Alliance> allianceOpt,
      String gameData,
      double secondsAhead) {
    if (!DriverStation.isTeleopEnabled() || Double.isNaN(teleopElapsedSec)) {
      return false;
    }

    return isHubActiveAtTeleopElapsed(teleopElapsedSec + secondsAhead, allianceOpt, gameData);
  }

  private boolean isPreActiveNow(
      double teleopElapsedSec,
      Optional<Alliance> allianceOpt,
      String gameData) {
    boolean activeNow = isHubActiveNow(teleopElapsedSec, allianceOpt, gameData);

    boolean activeInFiveSeconds =
        willBeActiveInSeconds(teleopElapsedSec, allianceOpt, gameData, kPreActiveWindowSec);

    return !activeNow && activeInFiveSeconds;
  }

  private boolean isFastPreActiveNow(
      double teleopElapsedSec,
      Optional<Alliance> allianceOpt,
      String gameData) {
    boolean activeNow = isHubActiveNow(teleopElapsedSec, allianceOpt, gameData);

    boolean activeInTwoSeconds =
        willBeActiveInSeconds(teleopElapsedSec, allianceOpt, gameData, kFastPreActiveWindowSec);

    return !activeNow && activeInTwoSeconds;
  }

  private void updateTimingState() {
    double now = Timer.getFPGATimestamp();

    boolean isFmsAttached = DriverStation.isFMSAttached();
    if (isFmsAttached && !wasFmsAttached) {
      fmsAttachTimestampSec = now;
    }

    if (!isFmsAttached) {
      fmsAttachTimestampSec = Double.NaN;
      teleopStartTimestampSec = Double.NaN;
    }

    boolean isTeleopEnabled = DriverStation.isTeleopEnabled();
    if (isTeleopEnabled && !wasTeleopEnabled) {
      teleopStartTimestampSec = now;
    }

    wasFmsAttached = isFmsAttached;
    wasTeleopEnabled = isTeleopEnabled;
  }

  private double getTeleopElapsedSec() {
    if (Double.isNaN(teleopStartTimestampSec)) {
      return Double.NaN;
    }

    return Math.max(0.0, Timer.getFPGATimestamp() - teleopStartTimestampSec);
  }

  private boolean isBlinkOn(double periodSec) {
    double now = Timer.getFPGATimestamp();
    double blinkOriginSec = !Double.isNaN(teleopStartTimestampSec)
        ? teleopStartTimestampSec
        : (Double.isNaN(fmsAttachTimestampSec) ? 0.0 : fmsAttachTimestampSec);
    double phaseSec = (now - blinkOriginSec) % periodSec;
    return phaseSec < (periodSec * 0.5);
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

    publishDashboardDefaults();
    updateTimingState();

    Optional<Alliance> dsAllianceOpt = DriverStation.getAlliance();
    Optional<Alliance> allianceOpt = getEffectiveAlliance(dsAllianceOpt);
    String gameData = DriverStation.getGameSpecificMessage();
    double teleopElapsedSec = getTeleopElapsedSec();

    boolean hubActive = isHubActiveNow(teleopElapsedSec, allianceOpt, gameData);
    boolean preActive = isPreActiveNow(teleopElapsedSec, allianceOpt, gameData);
    boolean fastPreActive = isFastPreActiveNow(teleopElapsedSec, allianceOpt, gameData);

    SmartDashboard.putBoolean("MatchHubActive", hubActive);
    SmartDashboard.putBoolean("MatchHubPreActive", preActive);
    SmartDashboard.putBoolean("MatchHubFastPreActive", fastPreActive);
    SmartDashboard.putBoolean("MatchHubFMSAttached", DriverStation.isFMSAttached());
    SmartDashboard.putNumber(
        "MatchHub/FMSAttachTimestampSec",
        Double.isNaN(fmsAttachTimestampSec) ? -1.0 : fmsAttachTimestampSec);
    SmartDashboard.putNumber(
        "MatchHub/TeleopElapsedSec",
        Double.isNaN(teleopElapsedSec) ? -1.0 : teleopElapsedSec);
    SmartDashboard.putBoolean("MatchHubRedActiveFirst", isRedActiveFirst(gameData));
    SmartDashboard.putNumber("MatchHubMatchTime", DriverStation.getMatchTime());
    SmartDashboard.putString("MatchHubGameData", gameData);
    SmartDashboard.putString(
        "MatchHubDSAlliance",
        dsAllianceOpt.isPresent() ? dsAllianceOpt.get().name() : "Unknown");
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

    if (fastPreActive) {
      if (isBlinkOn(kFastBlinkPeriodSec)) {
        setAll(teamR, teamG, teamB);
      } else {
        setAll(0, 0, 0);
      }
    } else if (preActive) {
      if (isBlinkOn(kBlinkPeriodSec)) {
        setAll(teamR, teamG, teamB);
      } else {
        setAll(0, 0, 0);
      }
    } else if (hubActive) {
      setAll(teamR, teamG, teamB);
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
