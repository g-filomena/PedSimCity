package pedsim.activity.engine;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import pedsim.core.engine.Exporter;
import pedsim.core.engine.FlowHandler;
import pedsim.core.utilities.StringEnum;

/**
 * The volumes file with its hours split into light and dark.
 *
 * <p>Adds LIGHT and DARK totals after the hour columns, and a pair per agent type where the file
 * carries agent types. Which hours are dark comes from {@link Daylight}, so the split follows the
 * same sunrise and sunset the agents walk by.
 *
 * <p>A seasonal split moves with the date: 17:00 is dark in December and light in June, so two runs
 * on different dates are comparable on the hour columns and not on these.
 */
public class ActivityExporter extends Exporter {

  public ActivityExporter(FlowHandler flowHandler, String appName) {
    super(flowHandler, appName);
  }

  @Override
  protected List<String> extraHourlyHeaders(Enum<?>[] agents, Enum<?>[] hours, boolean perAgent) {
    List<String> headers = new ArrayList<>();
    headers.add("LIGHT");
    headers.add("DARK");
    if (perAgent) {
      for (Enum<?> agent : agents) {
        headers.add(agent + "_LIGHT");
        headers.add(agent + "_DARK");
      }
    }
    return headers;
  }

  @Override
  protected List<String> extraHourlyValues(
      Map<String, Integer> edgeVolumes,
      Enum<?>[] agents,
      Enum<?>[] hours,
      boolean perAgent,
      int day) {
    List<String> values = new ArrayList<>();
    int light = 0;
    int dark = 0;
    for (Enum<?> agent : agents) {
      for (Enum<?> hour : hours) {
        int volume = cellVolume(edgeVolumes, agent, hour);
        if (isDarkColumn(hour, day)) {
          dark += volume;
        } else {
          light += volume;
        }
      }
    }
    values.add(Integer.toString(light));
    values.add(Integer.toString(dark));

    if (perAgent) {
      for (Enum<?> agent : agents) {
        int agentLight = 0;
        int agentDark = 0;
        for (Enum<?> hour : hours) {
          int volume = cellVolume(edgeVolumes, agent, hour);
          if (isDarkColumn(hour, day)) {
            agentDark += volume;
          } else {
            agentLight += volume;
          }
        }
        values.add(Integer.toString(agentLight));
        values.add(Integer.toString(agentDark));
      }
    }
    return values;
  }

  /** Whether an hour column is dark on the day being exported. */
  private boolean isDarkColumn(Enum<?> hour, int day) {
    return hour instanceof StringEnum.Hour
        && Daylight.isDarkHour(((StringEnum.Hour) hour).clockHour(), day);
  }
}
