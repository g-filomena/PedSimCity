package pedsim.core.agents;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.EnumSet;
import org.junit.jupiter.api.Test;
import pedsim.core.agents.RouteChoiceModel.BarrierPreferences;
import pedsim.core.agents.RouteChoiceModel.Strategy;
import pedsim.core.utilities.StringEnum.LandmarkType;
import pedsim.core.utilities.StringEnum.LocalHeuristicMode;
import pedsim.core.utilities.StringEnum.MinimisationMode;
import pedsim.core.utilities.StringEnum.RouteChoiceElement;

/** The rules a route-choice model enforces about itself. */
class RouteChoiceModelTest {

  @Test
  void pureMinimisationCarriesNoElements() {
    RouteChoiceModel model = RouteChoiceModel.minimisingDistance();

    assertSame(Strategy.PURE_MINIMISATION, model.strategy());
    assertSame(MinimisationMode.DISTANCE, model.minimisation());
    assertTrue(model.elements().isEmpty());
    assertTrue(model.isPureMinimisation());
  }

  @Test
  void elementBasedCarriesNoMinimisationMode() {
    RouteChoiceModel model = RouteChoiceModel.regions(LocalHeuristicMode.ANGULAR);

    assertSame(Strategy.ELEMENT_BASED, model.strategy());
    assertSame(MinimisationMode.NONE, model.minimisation());
    assertFalse(model.isPureMinimisation());
    assertTrue(model.hasElement(RouteChoiceElement.REGION_BASED_NAVIGATION));
  }

  /** The state that used to need a warning at plan time cannot be built. */
  @Test
  void aModelThatIsNeitherKindIsRefused() {
    assertThrows(
        IllegalArgumentException.class, () -> RouteChoiceModel.minimising(MinimisationMode.NONE));

    assertThrows(
        IllegalArgumentException.class,
        () ->
            RouteChoiceModel.usingElements(
                LocalHeuristicMode.NONE,
                EnumSet.noneOf(RouteChoiceElement.class),
                null,
                BarrierPreferences.NONE));
  }

  /** Recognising local landmarks is what gives a model a landmark type; distant ones do not. */
  @Test
  void onlyLocalLandmarksImplyALandmarkType() {
    assertSame(
        LandmarkType.LOCAL,
        RouteChoiceModel.localLandmarks(LocalHeuristicMode.DISTANCE).landmarkType());
    assertSame(
        LandmarkType.LOCAL,
        RouteChoiceModel.localAndDistantLandmarks(LocalHeuristicMode.DISTANCE).landmarkType());
    assertEquals(
        null, RouteChoiceModel.distantLandmarks(LocalHeuristicMode.DISTANCE).landmarkType());
  }

  /** A model asking for barrier sub-goals must be able to perceive a barrier. */
  @Test
  void barrierModelsCarryBarrierPerception() {
    for (RouteChoiceModel model :
        new RouteChoiceModel[] {
          RouteChoiceModel.barriers(LocalHeuristicMode.DISTANCE),
          RouteChoiceModel.regionsAndBarriers(LocalHeuristicMode.ANGULAR)
        }) {
      assertTrue(model.hasElement(RouteChoiceElement.BARRIER_BASED_NAVIGATION));
      assertEquals(
          BarrierPreferences.DEFAULT, model.barriers(), "barrier model with no perception");
    }

    assertSame(
        BarrierPreferences.NONE,
        RouteChoiceModel.regions(LocalHeuristicMode.DISTANCE).barriers(),
        "a region-only model should not perceive barriers");
  }

  /** The elements are copied in, so a caller cannot reach back into a built model. */
  @Test
  void elementsAreNotShared() {
    EnumSet<RouteChoiceElement> elements = EnumSet.of(RouteChoiceElement.REGION_BASED_NAVIGATION);
    RouteChoiceModel model =
        RouteChoiceModel.usingElements(
            LocalHeuristicMode.DISTANCE, elements, null, BarrierPreferences.NONE);

    elements.add(RouteChoiceElement.DISTANT_LANDMARKS);

    assertFalse(model.hasElement(RouteChoiceElement.DISTANT_LANDMARKS));
    assertThrows(
        UnsupportedOperationException.class,
        () -> model.elements().add(RouteChoiceElement.LOCAL_LANDMARKS));
  }
}
