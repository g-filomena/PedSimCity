package pedsim.core.parameters;

import java.awt.TextField;
import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import pedsim.core.applet.PedSimCityApplet;

/**
 * Central manager for simulation parameters. - Reflection-based (no manual binding) - Works with
 * CLI args, GUI panels, and Applets - Supports int, double, boolean, Integer[], Double[], String[]
 */
public class ParameterManager {

  // ------------------------------------------------------------
  // Apply values into static parameter classes
  // ------------------------------------------------------------

  /** Apply values from panel fields into a target parameter class. */
  public static void apply(Map<String, TextField> panelFields, Class<?> targetClass) {
    for (Map.Entry<String, TextField> entry : panelFields.entrySet())
      setFieldValue(targetClass, entry.getKey(), entry.getValue().getText().trim());
  }

  /** Apply multiple maps (e.g. doubles, booleans) to a class. */
  @SafeVarargs
  public static void applyAll(Class<?> targetClass, Map<String, TextField>... fieldMaps) {
    for (Map<String, TextField> map : fieldMaps)
      apply(map, targetClass);
  }

  /** Apply a parameter map (CLI style) to multiple target classes. */
  @SafeVarargs
  public static void applyParams(Map<String, String> params, Class<?>... targetClasses) {
    for (Map.Entry<String, String> e : params.entrySet())
      for (Class<?> cls : targetClasses)
        setFieldValue(cls, e.getKey(), e.getValue());
  }

  // ------------------------------------------------------------
  // CLI args → parameter classes
  // ------------------------------------------------------------

  /**
   * Parse CLI args and apply them to all parameter classes. Usable for both local CLI runs and
   * server/headless runs.
   */
  public static Map<String, String> initFromArgs(String[] args) {
    Map<String, String> params = parseArgs(args);
    applyParams(params, Pars.class, TimePars.class, RouteChoicePars.class);
    return params;
  }

  /** Legacy alias kept for compatibility (use initFromArgs instead). */
  @Deprecated(forRemoval = true)
  public static Map<String, String> initFromArgsForServer(String[] args) {
    return initFromArgs(args);
  }

  /** Directly set a parameter field by name. */
  public static void setFieldValue(Class<?> targetClass, String key, String raw) {
    if (raw == null || raw.isBlank())
      return;

    try {
      Field f = targetClass.getDeclaredField(key);
      f.setAccessible(true);
      Class<?> type = f.getType();

      if (type == int.class || type == Integer.class)
        f.set(null, (int) Double.parseDouble(raw));
      else if (type == double.class || type == Double.class)
        f.set(null, Double.parseDouble(raw));
      else if (type == boolean.class || type == Boolean.class)
        f.set(null, Boolean.parseBoolean(raw));
      else if (type == String.class)
        f.set(null, raw);
      else if (type == Integer[].class) {
        String[] parts = raw.split(",");
        Integer[] arr = new Integer[parts.length];
        for (int i = 0; i < parts.length; i++)
          arr[i] = Integer.parseInt(parts[i].trim());
        f.set(null, arr);
      } else if (type == Double[].class) {
        String[] parts = raw.split(",");
        Double[] arr = new Double[parts.length];
        for (int i = 0; i < parts.length; i++)
          arr[i] = Double.parseDouble(parts[i].trim());
        f.set(null, arr);
      } else if (type == String[].class) {
        String[] arr = raw.split(",");
        for (int i = 0; i < arr.length; i++)
          arr[i] = arr[i].trim();
        f.set(null, arr);
      } else {
        System.err
            .println("Unsupported type for field " + key + " in " + targetClass.getSimpleName());
      }
    } catch (NoSuchFieldException e) {
      System.err.println("No field named " + key + " in " + targetClass.getSimpleName());
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
        if (val == null)
          continue;

        String str;
        if (val.getClass().isArray()) {
          Object[] arr = (Object[]) val;
          StringBuilder sb = new StringBuilder();
          for (int i = 0; i < arr.length; i++) {
            sb.append(arr[i]);
            if (i < arr.length - 1)
              sb.append(",");
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
    for (Class<?> cls : classes)
      all.putAll(exportParams(cls));
    return all;
  }

  /** Convert params map to CLI string. */
  public static String toArgString(Map<String, String> params) {
    StringBuilder sb = new StringBuilder();
    for (Map.Entry<String, String> e : params.entrySet()) {
      sb.append("--").append(e.getKey());
      if (!"true".equals(e.getValue()))
        sb.append("=").append(e.getValue());
      sb.append(" ");
    }
    return sb.toString().trim();
  }

  // ------------------------------------------------------------
  // From Applet
  // ------------------------------------------------------------

  /**
   * Collect parameters from an Applet into CLI string.
   * 
   * @param headless if true, forces "--headless"
   */
  public static String collectParameters(PedSimCityApplet applet, boolean headless) {
    if (applet.getCityName() != null)
      setFieldValue(Pars.class, "cityName", applet.getCityName());
    if (applet.getDays() != null)
      setFieldValue(TimePars.class, "numberOfDays", applet.getDays());
    if (applet.getPopulation() != null)
      setFieldValue(Pars.class, "population", applet.getPopulation());
    if (applet.getPercentage() != null)
      setFieldValue(Pars.class, "percentagePopulationAgent", applet.getPercentage());
    if (applet.getJobs() != null)
      setFieldValue(Pars.class, "jobs", applet.getJobs());

    if (applet.getOtherParsPanel() != null)
      applyAll(Pars.class, applet.getOtherParsPanel().doubleFields,
          applet.getOtherParsPanel().booleanFields);

    if (applet.getRoutePanel() != null)
      applyAll(RouteChoicePars.class, applet.getRoutePanel().doubleFields,
          applet.getRoutePanel().booleanFields);

    Map<String, String> params = exportParams(Pars.class, TimePars.class, RouteChoicePars.class);
    if (headless)
      params.put("headless", "true");

    return toArgString(params);
  }

  /** Wrapper: collect parameters for local (non-headless) runs. */
  public static String collectParameters(PedSimCityApplet applet) {
    return collectParameters(applet, false);
  }

  /**
   * Wrapper: collect parameters specifically for server runs (forces headless).
   */
  public static String collectParametersForServerRun(PedSimCityApplet applet) {
    return collectParameters(applet, true);
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
        if (parts.length == 2)
          params.put(parts[0], parts[1]);
        else
          params.put(parts[0], "true");
      }
    }
    return params;
  }
}
