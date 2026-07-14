package pedsim.activity.agents;

import java.util.EnumSet;
import java.util.Locale;
import java.util.Map;
import java.util.Random;
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
  WORK(0.0, 24.0, 420, 0.20, 1.0),

  /** School / university attendance; destination is the agent's study node. */
  EDUCATION(0.0, 24.0, 330, 0.20, 1.0),

  SHOPPING(8.0, 20.0, 35, 0.55, 0.8),

  /** Personal business: bank, post office, pharmacy, GP… */
  ERRANDS(8.0, 18.0, 20, 0.50, 0.6),

  /** Restaurants, cafés, fast food. */
  DINING(11.0, 23.0, 75, 0.40, 0.9),

  /** Pubs, bars, clubs, cinemas, theatres; open into the small hours. */
  NIGHTLIFE(18.0, 2.0, 100, 0.45, 1.2),

  /** Parks, sport, culture, sights. */
  LEISURE(7.0, 23.0, 60, 0.55, 1.3),

  /** A walk for its own sake; always available, uniform destination choice. */
  STROLL(0.0, 24.0, 30, 0.40, 1.0);

  private final double openHour;
  private final double closeHour;
  private final double meanStayMinutes;
  private final double logSigma;
  private final double tripDistanceFactor;

  private static final double MIN_STAY_MINUTES = 5;
  private static final double MAX_STAY_MINUTES = 240;

  ActivityPurpose(
      double openHour,
      double closeHour,
      double meanStayMinutes,
      double logSigma,
      double tripDistanceFactor) {
    this.openHour = openHour;
    this.closeHour = closeHour;
    this.meanStayMinutes = meanStayMinutes;
    this.logSigma = logSigma;
    this.tripDistanceFactor = tripDistanceFactor;
  }

  /**
   * Purpose-typical scaling of the released trip distance: errands are run close to home,
   * nightlife and leisure justify longer walks. Applied to the distance band a discretionary
   * destination is sampled from.
   */
  public double getTripDistanceFactor() {
    return tripDistanceFactor;
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
  public int sampleStayMinutes(Random random) {
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
