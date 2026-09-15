package pedsim.cityimage.agents;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.EnumMap;
import java.util.Map;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import pedsim.cityimage.utilities.StringEnum.Scenario;
import pedsim.core.agents.RouteChoiceModel;
import pedsim.core.utilities.StringEnum.RouteChoiceElement;

/**
 * What each scenario means.
 *
 * <p>These exist because the module once derived a scenario's meaning from its name by testing for
 * substrings, and two scenarios silently resolved to one configuration. The compiler now forces
 * every constant to be given a model; these check that the models differ and say what their names
 * claim.
 */
class ScenarioModelTest {

  @ParameterizedTest
  @EnumSource(Scenario.class)
  void everyScenarioHasAModel(Scenario scenario) {
    assertTrue(CityImageAgent.modelFor(scenario) != null, scenario + " has no model");
  }

  /** Two scenarios that resolve to the same configuration are two names for one experiment. */
  @Test
  void noTwoScenariosShareAModel() {
    Map<RouteChoiceModel, Scenario> seen = new java.util.HashMap<>();
    for (Scenario scenario : Scenario.values()) {
      RouteChoiceModel model = CityImageAgent.modelFor(scenario);
      Scenario clash = seen.put(model, scenario);
      assertEquals(null, clash, () -> scenario + " is configured exactly like " + clash);
    }
  }

  /** A name that says LOCAL means local only, and one that says DISTANT means distant only. */
  @Test
  void landmarkNamesMatchTheirElements() {
    Map<Scenario, boolean[]> expected = new EnumMap<>(Scenario.class);
    // { local, distant }
    expected.put(Scenario.LOCAL_LANDMARKS_DISTANCE, new boolean[] {true, false});
    expected.put(Scenario.LOCAL_LANDMARKS_ANGULAR, new boolean[] {true, false});
    expected.put(Scenario.DISTANT_LANDMARKS_DISTANCE, new boolean[] {false, true});
    expected.put(Scenario.DISTANT_LANDMARKS_ANGULAR, new boolean[] {false, true});
    expected.put(Scenario.DISTANT_LANDMARKS, new boolean[] {false, true});
    expected.put(Scenario.LANDMARKS_DISTANCE, new boolean[] {true, true});
    expected.put(Scenario.LANDMARKS_ANGULAR, new boolean[] {true, true});

    expected.forEach(
        (scenario, flags) -> {
          RouteChoiceModel model = CityImageAgent.modelFor(scenario);
          assertEquals(
              flags[0],
              model.hasElement(RouteChoiceElement.LOCAL_LANDMARKS),
              scenario + ": local landmarks");
          assertEquals(
              flags[1],
              model.hasElement(RouteChoiceElement.DISTANT_LANDMARKS),
              scenario + ": distant landmarks");
        });
  }

  /** A name that says REGION or BARRIER carries that element, and one that does not, does not. */
  @ParameterizedTest
  @EnumSource(Scenario.class)
  void regionAndBarrierNamesMatchTheirElements(Scenario scenario) {
    RouteChoiceModel model = CityImageAgent.modelFor(scenario);
    String name = scenario.name();

    assertEquals(
        name.contains("REGION"),
        model.hasElement(RouteChoiceElement.REGION_BASED_NAVIGATION),
        scenario + ": region navigation");
    assertEquals(
        name.contains("BARRIER"),
        model.hasElement(RouteChoiceElement.BARRIER_BASED_NAVIGATION),
        scenario + ": barrier navigation");
  }

  /** The two baselines are the only pure minimisations; everything else is element-based. */
  @ParameterizedTest
  @EnumSource(Scenario.class)
  void onlyTheBaselinesArePureMinimisation(Scenario scenario) {
    boolean baseline = scenario == Scenario.ROAD_DISTANCE || scenario == Scenario.ANGULAR_CHANGE;
    assertEquals(
        baseline,
        CityImageAgent.modelFor(scenario).isPureMinimisation(),
        scenario + ": pure minimisation");
  }

  /** An element-based model whose elements are all dropped would route as its baseline. */
  @ParameterizedTest
  @EnumSource(Scenario.class)
  void elementBasedScenariosCarryAtLeastOneElement(Scenario scenario) {
    RouteChoiceModel model = CityImageAgent.modelFor(scenario);
    if (!model.isPureMinimisation()) {
      assertNotEquals(0, model.elements().size(), scenario + " names elements but has none");
    }
  }
}
