package pedsim.activity.agents;

import ec.util.MersenneTwisterFast;
import java.util.EnumSet;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import sim.util.geo.AttributeValue;

/**
 * The purpose of a discretionary activity, with its opening window and stay-duration model, plus
 * the OSM-style tag classification that maps POIs/buildings to purposes.
 *
 * <p>Opening windows give each purpose an "open hours" envelope: agendas only schedule (and chained
 * trips only start) activities whose window contains the current hour, which removes 4 AM shopping
 * trips without touching the release curve. Windows may wrap past midnight (e.g. nightlife 18–02).
 *
 * <p>Stay durations are lognormal around a purpose-typical mean — short errands, long nights out.
 */
public enum ActivityPurpose {

  /** Employment; destination is the agent's work node, duration handled by the persona. */
  WORK(0.0, 24.0, 420, 0.20),

  /** School / university attendance; destination is the agent's study node. */
  EDUCATION(0.0, 24.0, 330, 0.20),

  SHOPPING(8.0, 20.0, 35, 0.55),

  /** Personal business: bank, post office, pharmacy, GP… */
  ERRANDS(8.0, 18.0, 20, 0.50),

  /** Restaurants, cafés, fast food. */
  DINING(11.0, 23.0, 75, 0.40),

  /** Pubs, bars, clubs, cinemas, theatres; open into the small hours. */
  NIGHTLIFE(18.0, 2.0, 100, 0.45),

  /** Parks, sport, culture, sights. */
  LEISURE(7.0, 23.0, 60, 0.55),

  /** A walk for its own sake; always available, uniform destination choice. */
  STROLL(0.0, 24.0, 30, 0.40);

  // Not final: the constants above are generic defaults, replaceable per city through the
  // purpose.<NAME>.<setting> keys CityConfig reads. The defaults are kept alongside so a second city
  // in the same JVM does not inherit the first's hours.
  private double openHour;
  private double closeHour;
  private double meanStayMinutes;
  private double logSigma;

  private final double defaultOpenHour;
  private final double defaultCloseHour;
  private final double defaultMeanStayMinutes;
  private final double defaultLogSigma;

  private static final double MIN_STAY_MINUTES = 5;
  private static final double MAX_STAY_MINUTES = 240;

  ActivityPurpose(
      double openHour,
      double closeHour,
      double meanStayMinutes,
      double logSigma) {
    this.openHour = openHour;
    this.closeHour = closeHour;
    this.meanStayMinutes = meanStayMinutes;
    this.logSigma = logSigma;
    this.defaultOpenHour = openHour;
    this.defaultCloseHour = closeHour;
    this.defaultMeanStayMinutes = meanStayMinutes;
    this.defaultLogSigma = logSigma;
  }

  /**
   * Applies one {@code purpose.<NAME>.<setting>} key from a city file. Settings: {@code open},
   * {@code close} (hours; a close before an open wraps midnight), {@code stayMinutes} (lognormal
   * mean), {@code staySigma}.
   *
   * @param purposeName the enum constant's name, case-insensitive
   * @return whether the key named a purpose and a setting that exist
   */
  public static boolean applyCitySetting(String purposeName, String setting, double value) {
    ActivityPurpose purpose;
    try {
      purpose = valueOf(purposeName.toUpperCase(Locale.ROOT));
    } catch (IllegalArgumentException e) {
      return false;
    }
    switch (setting) {
      case "open":
        purpose.openHour = value;
        return true;
      case "close":
        purpose.closeHour = value;
        return true;
      case "stayMinutes":
        purpose.meanStayMinutes = value;
        return true;
      case "staySigma":
        purpose.logSigma = value;
        return true;
      default:
        return false;
    }
  }

  /** Restores every purpose's built-in window and stay duration. */
  public static void resetToDefaults() {
    for (ActivityPurpose purpose : values()) {
      purpose.openHour = purpose.defaultOpenHour;
      purpose.closeHour = purpose.defaultCloseHour;
      purpose.meanStayMinutes = purpose.defaultMeanStayMinutes;
      purpose.logSigma = purpose.defaultLogSigma;
    }
  }

  /** The window and stay duration as they stand, for logging what a city file did. */
  public String describeSettings() {
    return String.format(
        "%s open %.1f-%.1f, stay %.0f min (sigma %.2f)",
        name(), openHour, closeHour, meanStayMinutes, logSigma);
  }

  /*
   * There is deliberately no purpose scaling of trip distance here.
   *
   * A leg is as long as the walk to the place the agent chose, and purpose does not scale it, for
   * two reasons.
   *
   * The effect is small. Watson et al. (2021), 2017 NHTS, 54,034 walking trips, report walking
   * distances as not significantly different by purpose; the difference appears in duration, and the
   * whole spread across purposes is 1.15x.
   *
   * And a multiplier on a chosen distance is the wrong shape for it. Scaling each trip moves the
   * aggregate distribution unless the factors average exactly 1.0 over the purpose mix realised on
   * the day - a mix that shifts with the hour, the persona and the agenda, so it cannot be held
   * there. If purpose should drive distance, it belongs in destination choice, where purpose already
   * selects the attraction table: a per-purpose impedance coefficient rather than a multiplier. That
   * needs per-purpose walking trip-length distributions, which for Italy means the Audimob
   * microdata.
   *
   * Purpose still decides which node is chosen (POI weighting), how long the agent stays, and when
   * the activity is open at all.
   */

  /** Hour of day this activity opens. */
  public double getOpenHour() {
    return openHour;
  }

  /** Hour of day this activity closes; smaller than {@link #getOpenHour()} when it wraps midnight. */
  public double getCloseHour() {
    return closeHour;
  }

  /** Whether this activity can start at the given hour of day (window may wrap past midnight). */
  public boolean isOpenAt(double hourOfDay) {
    if (openHour <= closeHour) {
      return hourOfDay >= openHour && hourOfDay <= closeHour;
    }
    return hourOfDay >= openHour || hourOfDay <= closeHour; // wraps midnight
  }

  /**
   * Draws a stay duration in minutes: lognormal around the purpose-typical mean, clamped to
   * [5 min, 4 h]. WORK/EDUCATION durations are persona matters and are not drawn here.
   */
  public int sampleStayMinutes(MersenneTwisterFast random) {
    double draw = Math.exp(Math.log(meanStayMinutes) + logSigma * random.nextGaussian());
    return (int) Math.max(MIN_STAY_MINUTES, Math.min(MAX_STAY_MINUTES, draw));
  }

  // ----------------------------------------------------------------
  // OSM-style tag classification
  // ----------------------------------------------------------------

  /**
   * Attribute keys whose mere presence (any non-empty value) implies a purpose, following OSM
   * conventions: {@code shop=*} is a shop whatever it sells, {@code office=*} is a workplace.
   */
  private static ActivityPurpose purposeFromKey(String key) {
    return switch (key) {
      case "shop" -> SHOPPING;
      case "leisure", "tourism", "sport" -> LEISURE;
      case "office" -> WORK;
      default -> null;
    };
  }

  /**
   * Purpose implied by a tag *value* (checked across {@code amenity}, {@code use}, {@code fclass},
   * {@code type}, {@code building}, {@code landuse} and {@code land_use} keys — the last being the
   * scalar column the preparation pipeline writes on buildings). Values follow common OSM
   * vocabulary.
   */
  private static ActivityPurpose purposeFromValue(String value) {
    return switch (value) {
      case "restaurant", "cafe", "fast_food", "food_court", "ice_cream", "biergarten" -> DINING;
      case "bar", "pub", "nightclub", "casino", "cinema", "theatre", "music_venue" -> NIGHTLIFE;
      case "school", "university", "college", "kindergarten", "library", "education" -> EDUCATION;
      case "bank",
              "atm",
              "post_office",
              "pharmacy",
              "clinic",
              "doctors",
              "dentist",
              "hospital",
              "veterinary",
              "townhall",
              "courthouse",
              "police" ->
          ERRANDS;
      case "supermarket", "convenience", "mall", "department_store", "marketplace", "retail" ->
          SHOPPING;
      case "park",
              "garden",
              "playground",
              "pitch",
              "sports_centre",
              "fitness_centre",
              "swimming_pool",
              "stadium",
              "museum",
              "gallery",
              "attraction",
              "viewpoint",
              "zoo",
              "theme_park",
              "place_of_worship",
              "community_centre",
              "arts_centre" ->
          LEISURE;
      case "office", "commercial", "industrial", "government" -> WORK;
      // cityImage macro-group vocabulary (the labels in the buildings layer's land_use /
      // land_uses columns, produced by classify_land_uses_raws_into_OSMgroups).
      case "sustenance" -> DINING;
      case "healthcare", "financial", "public_service", "civic_amenity" -> ERRANDS;
      case "tourism", "leisure", "sports", "religious", "entertainment_arts_culture" -> LEISURE;
      default -> null;
    };
  }

  /** Keys whose presence alone classifies the feature. */
  private static final String[] KEY_TAGS = {"shop", "leisure", "tourism", "sport", "office"};

  /** Keys whose value is looked up in the shared vocabulary. */
  private static final String[] VALUE_TAGS = {
    "amenity", "use", "fclass", "type", "building", "landuse", "land_use"
  };

  /**
   * Classifies every recognised use of a feature: the single purpose from {@link #classify} plus
   * one purpose per label of the optional {@code land_uses} column — the full list of cityImage
   * macro-groups the preparation pipeline writes on buildings, stored in the GeoPackage as a
   * stringified Python list (e.g. {@code "['commercial', 'sustenance']"}). A mixed-use building
   * thus attracts every purpose it hosts, not only the first-listed one.
   */
  public static Set<ActivityPurpose> classifyAll(Map<String, AttributeValue> attributes) {
    Set<ActivityPurpose> purposes = EnumSet.noneOf(ActivityPurpose.class);
    if (attributes == null || attributes.isEmpty()) {
      return purposes;
    }
    ActivityPurpose scalar = classify(attributes);
    if (scalar != null) {
      purposes.add(scalar);
    }
    String rawList = tagValue(attributes, "land_uses");
    if (rawList != null && rawList.startsWith("[") && rawList.endsWith("]")) {
      for (String label : rawList.substring(1, rawList.length() - 1).split(",")) {
        String cleaned = label.trim().replaceAll("^['\"]|['\"]$", "");
        ActivityPurpose purpose = purposeFromValue(cleaned);
        if (purpose != null) {
          purposes.add(purpose);
        }
      }
    }
    return purposes;
  }

  /**
   * Classifies a feature's attribute map (OSM-like use tags) into an activity purpose, or
   * {@code null} when no recognised tag is present. Value-bearing keys ({@code amenity=pub}) are
   * checked before presence-only keys, so a specific value wins over a generic key.
   */
  public static ActivityPurpose classify(Map<String, AttributeValue> attributes) {
    if (attributes == null || attributes.isEmpty()) {
      return null;
    }

    for (String key : VALUE_TAGS) {
      String value = tagValue(attributes, key);
      if (value != null) {
        ActivityPurpose purpose = purposeFromValue(value);
        if (purpose != null) {
          return purpose;
        }
      }
    }
    for (String key : KEY_TAGS) {
      if (tagValue(attributes, key) != null) {
        ActivityPurpose purpose = purposeFromKey(key);
        if (purpose != null) {
          return purpose;
        }
      }
    }
    return null;
  }

  /** Normalised (trimmed, lower-case) tag value, or null when absent/blank/"no"/"yes". */
  private static String tagValue(Map<String, AttributeValue> attributes, String key) {
    AttributeValue attribute = attributes.get(key);
    if (attribute == null) {
      return null;
    }
    String raw;
    try {
      raw = attribute.getString();
    } catch (Exception e) {
      Object value = attribute.getValue();
      raw = value == null ? null : value.toString();
    }
    if (raw == null) {
      return null;
    }
    String value = raw.trim().toLowerCase(Locale.ROOT);
    // "yes" carries no category information (e.g. building=yes); "no"/empty mean absent.
    if (value.isEmpty() || value.equals("no") || value.equals("yes") || value.equals("none")) {
      return null;
    }
    return value;
  }
}
