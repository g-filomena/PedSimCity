package pedsim.core.agents;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ec.util.MersenneTwisterFast;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import pedsim.core.agents.RouteChoiceModel.BarrierPreferences;
import pedsim.core.engine.PedSimCity;
import pedsim.core.utilities.StringEnum.LocalHeuristicMode;
import pedsim.core.utilities.StringEnum.MinimisationMode;
import pedsim.core.utilities.StringEnum.RouteChoiceElement;

/**
 * The survey-style draw of {@link Heuristics}: elements or a pure minimisation, each mechanism only
 * where the city carries the data it needs.
 */
class HeuristicsActivationTest {

  private boolean landmarksWere;

  @BeforeEach
  void saveCityState() {
    landmarksWere = PedSimCity.landmarksLoaded;
  }

  @AfterEach
  void restoreCityState() {
    PedSimCity.landmarksLoaded = landmarksWere;
  }

  private static RouteChoiceActivation activation(double usingElements, double localLandmarks) {
    return new RouteChoiceActivation(
        usingElements, 1.0, 0.0, 1.0, 0.0, 0.0, localLandmarks, 0.0, 0.0, BarrierPreferences.NONE);
  }

  private static Heuristics heuristics(RouteChoiceActivation activation) {
    Heuristics heuristics = new Heuristics(new MersenneTwisterFast(42L));
    heuristics.setActivation(activation);
    return heuristics;
  }

  @Test
  void noActivationIsPureMinimisation() {
    RouteChoiceModel model = new Heuristics(new MersenneTwisterFast(1L)).defineHeuristic(false);
    assertTrue(model.isPureMinimisation());
  }

  @Test
  void anAgentWhoNeverUsesElementsMinimisesTheCostTheSurveySays() {
    PedSimCity.landmarksLoaded = true;
    for (int i = 0; i < 50; i++) {
      RouteChoiceModel model = heuristics(activation(0.0, 1.0)).defineHeuristic(false);
      assertTrue(model.isPureMinimisation());
      assertEquals(MinimisationMode.DISTANCE, model.minimisation());
    }
  }

  @Test
  void anAgentWhoAlwaysUsesElementsRecognisesLocalLandmarksWhereTheCityHasThem() {
    PedSimCity.landmarksLoaded = true;
    for (int i = 0; i < 50; i++) {
      RouteChoiceModel model = heuristics(activation(1.0, 1.0)).defineHeuristic(false);
      assertFalse(model.isPureMinimisation());
      assertTrue(model.elements().contains(RouteChoiceElement.LOCAL_LANDMARKS));
      assertEquals(LocalHeuristicMode.DISTANCE, model.localHeuristic());
      assertTrue(model.landmarkType() != null);
    }
  }

  @Test
  void aCityWithoutLandmarkScoresCannotUseThem() {
    PedSimCity.landmarksLoaded = false;
    for (int i = 0; i < 50; i++) {
      RouteChoiceModel model = heuristics(activation(1.0, 1.0)).defineHeuristic(false);
      assertTrue(model.isPureMinimisation());
    }
  }
}
