package pedsim.night.engine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.night.parameters.NightPars;

class EyesOnStreetTest {

  private final double saturation = NightPars.eyesSaturationPer100m;

  @AfterEach
  void restore() {
    NightPars.eyesSaturationPer100m = saturation;
  }

  @Test
  void aStreetWithNoOpenVenueHasNoEyes() {
    assertEquals(0.0, EyesOnStreet.eyes(0.0, 120.0));
  }

  @Test
  void eyesReachOneMinusOneOverEAtTheSaturationDensity() {
    NightPars.eyesSaturationPer100m = 2.0;
    assertEquals(1.0 - Math.exp(-1.0), EyesOnStreet.eyes(4.0, 200.0), 1e-12);
  }

  @Test
  void eyesRiseWithVenuesAndStayBelowOne() {
    NightPars.eyesSaturationPer100m = 2.0;
    double previous = 0.0;
    for (int venues = 1; venues <= 40; venues++) {
      double eyes = EyesOnStreet.eyes(venues, 100.0);
      assertTrue(eyes > previous, "rises at " + venues);
      assertTrue(eyes < 1.0, "below one at " + venues);
      previous = eyes;
    }
  }

  @Test
  void aShortStretchIsMeasuredAsIfItWereFiftyMetresLong() {
    NightPars.eyesSaturationPer100m = 2.0;
    assertEquals(EyesOnStreet.eyes(1.0, 50.0), EyesOnStreet.eyes(1.0, 5.0), 1e-12);
    assertTrue(EyesOnStreet.eyes(1.0, 5.0) < 0.7, "one bar on a corner is not a lively street");
  }

  @Test
  void onlyVenuesWithOpeningHoursCount() {
    assertTrue(EyesOnStreet.PURPOSES.contains(ActivityPurpose.NIGHTLIFE));
    assertTrue(EyesOnStreet.PURPOSES.contains(ActivityPurpose.DINING));
    // Workplaces and schools are open 0-24 in the enum because the persona decides their hours,
    // so counting them would put eyes on every office street all night.
    assertTrue(!EyesOnStreet.PURPOSES.contains(ActivityPurpose.WORK));
    assertTrue(!EyesOnStreet.PURPOSES.contains(ActivityPurpose.EDUCATION));
  }
}
