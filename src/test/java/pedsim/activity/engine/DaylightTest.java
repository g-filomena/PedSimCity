package pedsim.activity.engine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.time.LocalDateTime;
import org.junit.jupiter.api.Test;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.parameters.Pars;
import pedsim.testing.ParameterSnapshot;

/**
 * Pins the two regimes of {@link Daylight}: darkness follows the city's position and the season
 * where a position is known, and a stated fixed window where none is. The simulation cannot check
 * either about itself - a sunset computed for the wrong place, or in the wrong time zone, is a
 * perfectly plausible sunset.
 */
public class DaylightTest {

  /** Turin, as the network centre reports it. */
  private static final double TURIN_LAT = 45.0691;

  private static final double TURIN_LON = 7.6869;

  private static final LocalDateTime JUNE_EVENING = LocalDateTime.of(2026, 6, 21, 21, 0);
  private static final LocalDateTime DECEMBER_EVENING = LocalDateTime.of(2026, 12, 21, 17, 30);

  @Test
  void sunsetMatchesThePublishedTimesForTurin() throws Exception {
    try (var saved = new ParameterSnapshot(ActivityPars.class, Pars.class)) {
      turin();
      // Published: 21:18 on 21 June, 16:52 on 21 December (Turin, local clock).
      assertEquals(21.30, Daylight.sunriseSunsetHours(JUNE_EVENING)[1], 0.1);
      assertEquals(16.87, Daylight.sunriseSunsetHours(DECEMBER_EVENING)[1], 0.1);
      // Published sunrise: 05:43 in June, 08:03 in December.
      assertEquals(5.72, Daylight.sunriseSunsetHours(JUNE_EVENING)[0], 0.1);
      assertEquals(8.05, Daylight.sunriseSunsetHours(DECEMBER_EVENING)[0], 0.1);
    }
  }

  @Test
  void withAPositionDarknessFollowsTheSeason() throws Exception {
    try (var saved = new ParameterSnapshot(ActivityPars.class, Pars.class)) {
      turin();
      assertTrue(Daylight.hasPosition());
      assertFalse(Daylight.isDark(JUNE_EVENING), "Turin is still light at 21:00 in June");
      assertTrue(Daylight.isDark(DECEMBER_EVENING), "Turin is dark by 17:30 in December");
      assertTrue(Daylight.describeRegime().contains("sunrise/sunset"));
    }
  }

  @Test
  void withoutATimeZoneSummerEveningsArriveAnHourEarly() throws Exception {
    try (var saved = new ParameterSnapshot(ActivityPars.class, Pars.class)) {
      turin();
      double withZone = Daylight.sunriseSunsetHours(JUNE_EVENING)[1];
      ActivityPars.timeZoneId = "";
      double withoutZone = Daylight.sunriseSunsetHours(JUNE_EVENING)[1];
      assertEquals(1.0, withZone - withoutZone, 0.02);
    }
  }

  @Test
  void withoutAPositionTheFixedWindowDecidesAndHasNoSeason() throws Exception {
    try (var saved = new ParameterSnapshot(ActivityPars.class, Pars.class)) {
      Pars.cityLatitude = Double.NaN;
      Pars.cityLongitude = Double.NaN;
      assertFalse(Daylight.hasPosition());
      // 21:00 and 17:30 sit on opposite sides of the fixed boundary, whatever the season.
      assertTrue(Daylight.isDark(JUNE_EVENING));
      assertTrue(Daylight.isDark(JUNE_EVENING.withMonth(12)));
      assertFalse(Daylight.isDark(DECEMBER_EVENING));
      assertFalse(Daylight.isDark(DECEMBER_EVENING.withMonth(6)));
      assertTrue(Daylight.isDark(LocalDateTime.of(2026, 6, 21, 5, 0)));
      assertTrue(Daylight.describeRegime().contains("fixed window"));
    }
  }

  @Test
  void theFixedWindowIsAnInputNotAConstant() throws Exception {
    try (var saved = new ParameterSnapshot(ActivityPars.class, Pars.class)) {
      Pars.cityLatitude = Double.NaN;
      Pars.cityLongitude = Double.NaN;
      ActivityPars.dayStartHour = 8.0;
      ActivityPars.nightStartHour = 22.0;
      assertTrue(Daylight.isDark(LocalDateTime.of(2026, 6, 21, 7, 30)));
      assertFalse(Daylight.isDark(LocalDateTime.of(2026, 6, 21, 8, 30)));
      assertFalse(Daylight.isDark(JUNE_EVENING));
      assertTrue(Daylight.isDark(JUNE_EVENING.withHour(22)));
    }
  }

  private void turin() {
    Pars.cityLatitude = TURIN_LAT;
    Pars.cityLongitude = TURIN_LON;
    ActivityPars.timeZoneId = "Europe/Rome";
    ActivityPars.twilightBufferMinutes = 30;
    ActivityPars.dayStartHour = 6.0;
    ActivityPars.nightStartHour = 20.0;
  }
}
