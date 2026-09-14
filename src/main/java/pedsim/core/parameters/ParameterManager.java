package pedsim.core.parameters;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import pedsim.core.utilities.LoggerUtil;

/**
 * Central manager for simulation parameters. - Reflection-based (no manual binding) - Works with
 * CLI args, GUI panels, and Applets - Supports int, double, boolean, Integer[], Double[], String[]
 */
public class ParameterManager {

  // ------------------------------------------------------------
  // Apply values into static parameter classes
  // ------------------------------------------------------------

  // Parameters reach the model from exactly two places, both of which leave a record a run can be
  // reproduced from: the running module's per-city file, and the command line.

  /** Apply a parameter map (CLI style) to multiple target classes. */
  @SafeVarargs
  public static void applyParams(Map<String, String> params, Class<?>... targetClasses) {
    for (Map.Entry<String, String> e : params.entrySet())
      for (Class<?> cls : targetClasses) setFieldValue(cls, e.getKey(), e.getValue());
  }

  // ------------------------------------------------------------
  // CLI args → parameter classes
  // ------------------------------------------------------------

  // One entry point, and it takes the running module's own parameterClasses(). A variant reaching
  // only core's three classes would accept a module's key on the command line and then ignore it.

  /**
   * Parse CLI args and apply them to every parameter class the running module declares.
   *
   * @param args the raw command line
   * @param targets every class a value may be written into, core's first
   */
  public static Map<String, String> initFromArgs(String[] args, Class<?>[] targets) {
    Map<String, String> params = parseArgs(args);
    applyParams(params, targets);
    if (params.containsKey("percentage")) {
      Pars.percentagePopulationAgent = Double.parseDouble(params.get("percentage"));
    }
    if (params.containsKey("actualPopulation")) {
      Pars.population = Integer.parseInt(params.get("actualPopulation"));
    }
    if (params.containsKey("days")) {
      Pars.durationDays = Integer.parseInt(params.get("days"));
    }
    Pars.recomputeAgentCount();
    // Asking for a circuity factor means asking for that factor: NetworkCircuity measures into the
    // same field and consults only measureNetworkCircuity, so supplying one without the other would
    // have the measurement overwrite it at startup. An explicit measureNetworkCircuity on the same
    // command line still wins.
    if (params.containsKey("networkCircuityFactor") && !params.containsKey("measureNetworkCircuity")) {
      Pars.measureNetworkCircuity = false;
    }
    return params;
  }

  /** Directly set a parameter field by name. */
  public static void setFieldValue(Class<?> targetClass, String key, String raw) {
    if (raw == null || raw.isBlank()) return;

    try {
      Field f = targetClass.getDeclaredField(key);
      f.setAccessible(true);
      Class<?> type = f.getType();

      if (type == int.class || type == Integer.class) f.set(null, (int) Double.parseDouble(raw));
      else if (type == double.class || type == Double.class) f.set(null, Double.parseDouble(raw));
      else if (type == boolean.class || type == Boolean.class)
        f.set(null, Boolean.parseBoolean(raw));
      else if (type == String.class) f.set(null, raw);
      else if (type == Integer[].class) {
        String[] parts = raw.split(",");
        Integer[] arr = new Integer[parts.length];
        for (int i = 0; i < parts.length; i++) arr[i] = Integer.parseInt(parts[i].trim());
        f.set(null, arr);
      } else if (type == Double[].class) {
        String[] parts = raw.split(",");
        Double[] arr = new Double[parts.length];
        for (int i = 0; i < parts.length; i++) arr[i] = Double.parseDouble(parts[i].trim());
        f.set(null, arr);
      } else if (type == java.time.LocalDate.class) {
        // ISO-8601, e.g. --SIMULATION_START_DATE=2026-12-07. The date sets day-of-week and
        // day-of-year, so it chooses the season a run happens in.
        f.set(null, java.time.LocalDate.parse(raw.trim()));
      } else if (type == String[].class) {
        String[] arr = raw.split(",");
        for (int i = 0; i < arr.length; i++) arr[i] = arr[i].trim();
        f.set(null, arr);
      } else {
        System.err.println(
            "Unsupported type for field " + key + " in " + targetClass.getSimpleName());
      }
    } catch (NoSuchFieldException e) {
      // Every key is offered to each parameter class in turn, so misses are the norm, not a
      // problem: one hit is what matters. Reporting them on stderr buried every run start.
      LoggerUtil.getLogger()
          .fine("No field named " + key + " in " + targetClass.getSimpleName());
    } catch (Exception e) {
      System.err.println("Failed to set field " + key + ": " + e.getMessage());
    }
  }

  // ------------------------------------------------------------
  // Export to CLI args
  // ------------------------------------------------------------

  /** Convert all static fields of a class into a CLI param map. */
  public static Map<String, String> exportParams(Class<?> targetClass) {
    Map<String, String> params = new LinkedHashMap<>();
    for (Field f : targetClass.getDeclaredFields()) {
      try {
        f.setAccessible(true);
        Object val = f.get(null);
        if (val == null) continue;

        String str;
        if (val.getClass().isArray()) {
          Object[] arr = (Object[]) val;
          StringBuilder sb = new StringBuilder();
          for (int i = 0; i < arr.length; i++) {
            sb.append(arr[i]);
            if (i < arr.length - 1) sb.append(",");
          }
          str = sb.toString();
        } else {
          str = val.toString();
        }
        params.put(f.getName(), str);
      } catch (Exception e) {
        System.err.println("Failed to export " + f.getName() + ": " + e.getMessage());
      }
    }
    return params;
  }

  /** Merge multiple classes' parameters into one map. */
  @SafeVarargs
  public static Map<String, String> exportParams(Class<?>... classes) {
    Map<String, String> all = new LinkedHashMap<>();
    for (Class<?> cls : classes) all.putAll(exportParams(cls));
    return all;
  }

  /** Convert params map to CLI string. */
  public static String toArgString(Map<String, String> params) {
    StringBuilder sb = new StringBuilder();
    for (Map.Entry<String, String> e : params.entrySet()) {
      sb.append("--").append(e.getKey());
      if (!"true".equals(e.getValue())) sb.append("=").append(e.getValue());
      sb.append(" ");
    }
    return sb.toString().trim();
  }

  // ------------------------------------------------------------
  // CLI arg parsing
  // ------------------------------------------------------------

  /** Parse CLI args like --key=value into a param map. */
  public static Map<String, String> parseArgs(String[] args) {
    Map<String, String> params = new HashMap<>();
    for (String arg : args) {
      if (arg.startsWith("--")) {
        String[] parts = arg.substring(2).split("=", 2);
        if (parts.length == 2) params.put(parts[0], parts[1]);
        else params.put(parts[0], "true");
      }
    }
    return params;
  }
}
