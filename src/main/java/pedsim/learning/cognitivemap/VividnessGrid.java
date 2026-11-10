package pedsim.learning.cognitivemap;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.function.DoubleUnaryOperator;
import org.locationtech.jts.algorithm.locate.IndexedPointInAreaLocator;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Location;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.geom.Polygon;
import org.locationtech.jts.operation.union.UnaryUnionOp;
import pedsim.learning.parameters.LearningPars;

/**
 * A raster of how strongly an agent remembers each part of the city.
 *
 * <p>A cell holds the number of walked routes that let the agent see it, in two stores. The labile
 * store takes every walk and fades. The consolidated store takes {@code
 * LearningPars.consolidatedShare} of a walk, but only the first to reach the cell on a given day,
 * and fades far more slowly, by {@link #decayConsolidated}. A cell's strength is the sum of the
 * two, so a place seen on one day is eventually left with a floor, and a place seen on several
 * days with a floor above the known-space threshold.
 *
 * <p>Fading is a single scale factor over the labile store, so a step costs the same however large
 * the raster is; the factor is folded into the cells when it gets small.
 *
 * <p>Everything this class returns is in real units, that is, with the scale applied.
 */
public class VividnessGrid {

  /** A cell weaker than this is forgotten outright. */
  private static final double FORGOTTEN = 1e-6;

  /** The scale is folded into the cells once it falls below this. */
  private static final double FOLD_BELOW = 1e-3;

  /** Weight of a route's own contribution to a cell: one walk. */
  private static final double ONE_WALK = 1.0;

  private static final GeometryFactory GEOMETRY_FACTORY = new GeometryFactory();

  private int width;
  private int height;
  private final double cellSize;
  private double originX;
  private double originY;

  /** The labile store as stored; its real value is {@code cells[i] * scale}. */
  private float[] cells;

  /** The consolidated store: what the walks left that fades only slowly. */
  private float[] consolidated;

  /** For each cell, one more than the day of the last deposit in the consolidated store. */
  private int[] consolidatedOn;

  /** The simulated day the walks being added belong to. */
  private long day = 0;

  private double scale = 1.0;

  /** The threshold of the last collage, in real units. */
  private double collageThreshold = 0.0;

  /**
   * How close the last collage's most fadable cell is to the threshold: the smallest stored labile
   * value over what that cell needs from it to reach the threshold. The collage has faded once this
   * times the scale is below one. Infinite when no cell of the collage can fade.
   */
  private double weakestRatio = Double.POSITIVE_INFINITY;

  /** Half-width, in cells, of the sampling disc at each row offset from its centre. */
  private final int[] discHalfWidth;

  private final int discRadiusCells;
  private final int discCellCount;

  public VividnessGrid(Envelope envelope, double cellSize) {
    this.cellSize = cellSize;
    this.originX = envelope.getMinX();
    this.originY = envelope.getMinY();
    this.width = Math.max(1, (int) Math.ceil((envelope.getMaxX() - originX) / cellSize));
    this.height = Math.max(1, (int) Math.ceil((envelope.getMaxY() - originY) / cellSize));
    this.cells = new float[width * height];
    this.consolidated = new float[width * height];
    this.consolidatedOn = new int[width * height];

    double radiusCells = LearningPars.RouteVividnessRadius / cellSize;
    this.discRadiusCells = (int) Math.ceil(radiusCells);
    this.discHalfWidth = new int[2 * discRadiusCells + 1];
    int count = 0;
    for (int dy = -discRadiusCells; dy <= discRadiusCells; dy++) {
      double rest = radiusCells * radiusCells - (double) dy * dy;
      int half = rest < 0 ? -1 : (int) Math.floor(Math.sqrt(rest));
      discHalfWidth[dy + discRadiusCells] = half;
      count += 2 * half + 1;
    }
    this.discCellCount = count;
  }

  /** A grid holding exactly what this one holds, that changes independently of it. */
  public VividnessGrid copy() {
    VividnessGrid copy =
        new VividnessGrid(
            new Envelope(originX, originX + width * cellSize, originY, originY + height * cellSize),
            cellSize);
    copy.cells = cells.clone();
    copy.consolidated = consolidated.clone();
    copy.consolidatedOn = consolidatedOn.clone();
    copy.day = day;
    copy.scale = scale;
    copy.collageThreshold = collageThreshold;
    copy.weakestRatio = weakestRatio;
    return copy;
  }

  // ---------------------------------------------------------------------------------------------
  // Writing
  // ---------------------------------------------------------------------------------------------

  /** Sets the simulated day the walks added from now on belong to. */
  public void setDay(long day) {
    this.day = day;
  }

  /**
   * Adds {@code weight} to every cell whose centre lies inside the polygon, growing the raster to
   * take in a polygon that reaches beyond it.
   */
  public void addVisibilitySpace(Polygon polygon, double weight) {
    deposit(cellsInside(polygon), weight);
  }

  /** Adds one walk's worth of memory to the cells inside the polygon. */
  public void addVisibilitySpace(Polygon polygon) {
    addVisibilitySpace(polygon, ONE_WALK);
  }

  /**
   * Adds a walk whose weight depends on how new its space is to the agent.
   *
   * <p>The novelty is one minus the mean of {@code w / (1 + w)} over the cells inside the polygon,
   * measured before the walk is added: 1 for a space never seen, near 0 for one remembered well.
   *
   * @param weightFromNovelty the weight of the walk, given that novelty
   * @return the novelty
   */
  public double stamp(Polygon polygon, DoubleUnaryOperator weightFromNovelty) {
    int[] inside = cellsInside(polygon);
    if (inside.length == 0) {
      return 1.0;
    }
    double seen = 0.0;
    for (int i : inside) {
      double w = strength(i);
      seen += w / (1.0 + w);
    }
    double novelty = 1.0 - seen / inside.length;
    deposit(inside, weightFromNovelty.applyAsDouble(novelty));
    return novelty;
  }

  /**
   * Splits a walk's weight between the two stores of every listed cell. A cell that already took a
   * consolidated deposit today gets the labile part only.
   */
  private void deposit(int[] cellIndices, double weight) {
    double share = Math.max(0.0, Math.min(1.0, LearningPars.consolidatedShare));
    float labile = (float) ((1.0 - share) * weight / scale);
    float lasting = (float) (share * weight);
    int today = (int) day + 1;
    for (int i : cellIndices) {
      cells[i] += labile;
      if (consolidatedOn[i] != today) {
        consolidated[i] += lasting;
        consolidatedOn[i] = today;
      }
    }
  }

  /** The indices of the cells whose centre lies inside the polygon, growing the raster to fit. */
  private int[] cellsInside(Polygon polygon) {
    Envelope polyEnv = polygon.getEnvelopeInternal();
    growToInclude(polyEnv);

    IndexedPointInAreaLocator locator = new IndexedPointInAreaLocator(polygon);
    Coordinate centre = new Coordinate();

    int minX = Math.max(0, toGridX(polyEnv.getMinX()));
    int maxX = Math.min(width - 1, toGridX(polyEnv.getMaxX()));
    int minY = Math.max(0, toGridY(polyEnv.getMinY()));
    int maxY = Math.min(height - 1, toGridY(polyEnv.getMaxY()));

    int[] found = new int[256];
    int n = 0;
    for (int y = minY; y <= maxY; y++) {
      centre.y = originY + (y + 0.5) * cellSize;
      for (int x = minX; x <= maxX; x++) {
        centre.x = originX + (x + 0.5) * cellSize;
        if (locator.locate(centre) == Location.INTERIOR) {
          if (n == found.length) found = Arrays.copyOf(found, n * 2);
          found[n++] = y * width + x;
        }
      }
    }
    return Arrays.copyOf(found, n);
  }

  /**
   * Grows the raster, on its own lattice, until it contains the envelope. Cells keep their place
   * in the world; the new origin is a whole number of cells from the old one.
   */
  private void growToInclude(Envelope env) {
    double maxX = originX + width * cellSize;
    double maxY = originY + height * cellSize;
    if (env.getMinX() >= originX
        && env.getMinY() >= originY
        && env.getMaxX() <= maxX
        && env.getMaxY() <= maxY) {
      return;
    }

    int shiftX =
        env.getMinX() < originX ? (int) Math.ceil((originX - env.getMinX()) / cellSize) : 0;
    int shiftY =
        env.getMinY() < originY ? (int) Math.ceil((originY - env.getMinY()) / cellSize) : 0;
    double newOriginX = originX - shiftX * cellSize;
    double newOriginY = originY - shiftY * cellSize;
    double newMaxX = Math.max(maxX, env.getMaxX());
    double newMaxY = Math.max(maxY, env.getMaxY());
    int newWidth = (int) Math.ceil((newMaxX - newOriginX) / cellSize);
    int newHeight = (int) Math.ceil((newMaxY - newOriginY) / cellSize);

    float[] grown = new float[newWidth * newHeight];
    float[] grownLasting = new float[newWidth * newHeight];
    int[] grownOn = new int[newWidth * newHeight];
    for (int y = 0; y < height; y++) {
      System.arraycopy(cells, y * width, grown, (y + shiftY) * newWidth + shiftX, width);
      System.arraycopy(
          consolidated, y * width, grownLasting, (y + shiftY) * newWidth + shiftX, width);
      System.arraycopy(consolidatedOn, y * width, grownOn, (y + shiftY) * newWidth + shiftX, width);
    }
    this.width = newWidth;
    this.height = newHeight;
    this.originX = newOriginX;
    this.originY = newOriginY;
    this.cells = grown;
    this.consolidated = grownLasting;
    this.consolidatedOn = grownOn;
  }

  // ---------------------------------------------------------------------------------------------
  // Fading
  // ---------------------------------------------------------------------------------------------

  /**
   * Fades every cell by {@code factor}, in constant time.
   *
   * @return the number of cells forgotten outright by this call, that is, left with nothing in
   *     either store; non-zero only on the calls that fold the scale into the cells
   */
  public int decay(double factor) {
    scale *= factor;
    return scale < FOLD_BELOW ? fold() : 0;
  }

  /**
   * Fades the consolidated store by {@code factor}, cell by cell. The collage kept by {@link
   * #updateCollage} is out of date after this; the caller builds it again.
   */
  public void decayConsolidated(double factor) {
    for (int i = 0; i < consolidated.length; i++) {
      if (consolidated[i] == 0f) continue;
      double value = consolidated[i] * factor;
      consolidated[i] = value < FORGOTTEN ? 0f : (float) value;
    }
  }

  /** Applies the scale to the labile store and drops the cells that have become negligible. */
  private int fold() {
    int forgotten = 0;
    for (int i = 0; i < cells.length; i++) {
      if (cells[i] == 0f) continue;
      double value = cells[i] * scale;
      if (value < FORGOTTEN) {
        cells[i] = 0f;
        if (consolidated[i] == 0f) forgotten++;
      } else {
        cells[i] = (float) value;
      }
    }
    weakestRatio *= scale;
    scale = 1.0;
    return forgotten;
  }

  // ---------------------------------------------------------------------------------------------
  // Reading
  // ---------------------------------------------------------------------------------------------

  public double getCellSize() {
    return cellSize;
  }

  public int toGridX(double worldX) {
    return (int) Math.floor((worldX - originX) / cellSize);
  }

  public int toGridY(double worldY) {
    return (int) Math.floor((worldY - originY) / cellSize);
  }

  public boolean isInside(int gx, int gy) {
    return gx >= 0 && gy >= 0 && gx < width && gy < height;
  }

  /** The strength of the cell at a gridpoint; 0 outside the raster. */
  public double getDensityAtCell(int gx, int gy) {
    return isInside(gx, gy) ? strength(gy * width + gx) : 0.0;
  }

  /** The strength of the cell containing a point; 0 outside the raster. */
  public double getValueAt(double worldX, double worldY) {
    return getDensityAtCell(toGridX(worldX), toGridY(worldY));
  }

  /** The strength of a cell by index, in walks: both stores. */
  private double strength(int index) {
    return cells[index] * scale + consolidated[index];
  }

  /** The number of cells that hold any memory. */
  public int rememberedCells() {
    int n = 0;
    for (int i = 0; i < cells.length; i++) {
      if (cells[i] > 0f || consolidated[i] > 0f) n++;
    }
    return n;
  }

  /**
   * The remembered cells at or above {@code minThreshold}, as polygons: horizontal runs of cells
   * become rectangles, and the rectangles are unioned into contiguous regions.
   *
   * <p>The threshold and how near its most fadable cell is to it are kept, so {@link
   * #collageHasFaded()} can say when a fade has taken a cell of this collage below the threshold.
   * A cell whose consolidated store alone reaches the threshold never fades below it.
   *
   * @param minThreshold the least strength a cell must have to count
   * @return one polygon per contiguous region; empty when no cell qualifies
   */
  public List<Polygon> updateCollage(double minThreshold) {
    collageThreshold = minThreshold;
    weakestRatio = Double.POSITIVE_INFINITY;
    List<Polygon> runs = new ArrayList<>();

    for (int y = 0; y < height; y++) {
      int rowStart = y * width;
      int x = 0;
      while (x < width) {
        if (!qualifies(rowStart + x, minThreshold)) {
          x++;
          continue;
        }
        int first = x;
        while (x < width && qualifies(rowStart + x, minThreshold)) {
          double needed = minThreshold - consolidated[rowStart + x];
          if (needed > 0.0) {
            weakestRatio = Math.min(weakestRatio, cells[rowStart + x] / needed);
          }
          x++;
        }
        runs.add(rectangle(first, y, x, y + 1));
      }
    }
    if (runs.isEmpty()) return Collections.emptyList();

    Geometry unioned = UnaryUnionOp.union(runs);
    List<Polygon> result = new ArrayList<>();
    if (unioned instanceof Polygon polygon) {
      result.add(polygon);
    } else if (unioned instanceof MultiPolygon multi) {
      for (int i = 0; i < multi.getNumGeometries(); i++) {
        result.add((Polygon) multi.getGeometryN(i));
      }
    }
    return result;
  }

  private boolean qualifies(int index, double threshold) {
    double value = strength(index);
    return value > 0.0 && value >= threshold;
  }

  /**
   * Whether fading has taken a cell of the last collage below that collage's threshold, so that the
   * collage is no longer what {@link #updateCollage} would return now. False before any collage and
   * after an empty one. Cells stamped since the collage are not looked at: writing is the caller's
   * cue to build it again.
   */
  public boolean collageHasFaded() {
    return weakestRatio * scale < 1.0;
  }

  private Polygon rectangle(int x0, int y0, int x1, int y1) {
    double minX = originX + x0 * cellSize;
    double minY = originY + y0 * cellSize;
    double maxX = originX + x1 * cellSize;
    double maxY = originY + y1 * cellSize;
    return GEOMETRY_FACTORY.createPolygon(
        new Coordinate[] {
          new Coordinate(minX, minY),
          new Coordinate(maxX, minY),
          new Coordinate(maxX, maxY),
          new Coordinate(minX, maxY),
          new Coordinate(minX, minY)
        });
  }

  // ---------------------------------------------------------------------------------------------
  // Vividness of a space
  // ---------------------------------------------------------------------------------------------

  /**
   * How well the agent knows the space between two places, in [0, 1].
   *
   * <p>Discs of radius {@code LearningPars.RouteVividnessRadius} are laid around both places and at
   * most one radius apart along the line between them. In each, every cell contributes {@code w / (1
   * + w)} for its strength {@code w} in walks, so a cell walked once counts one half and the
   * contribution saturates towards one with repetition; the disc's vividness is that sum over all
   * its cells, remembered or not. The result is the mean over the discs. It depends only on the grid
   * and the two places.
   */
  public double vividnessBetween(Coordinate origin, Coordinate destination) {
    double sum = discVividness(origin.x, origin.y) + discVividness(destination.x, destination.y);
    int discs = 2;

    double dx = destination.x - origin.x;
    double dy = destination.y - origin.y;
    int steps = (int) Math.ceil(Math.hypot(dx, dy) / LearningPars.RouteVividnessRadius);
    for (int i = 1; i < steps; i++) {
      double t = i / (double) steps;
      sum += discVividness(origin.x + t * dx, origin.y + t * dy);
      discs++;
    }
    return sum / discs;
  }

  /** The mean contribution of the cells of one disc, outside-the-raster cells counting as zero. */
  private double discVividness(double x, double y) {
    int cx = toGridX(x);
    int cy = toGridY(y);
    double total = 0.0;

    for (int dy = -discRadiusCells; dy <= discRadiusCells; dy++) {
      int gy = cy + dy;
      if (gy < 0 || gy >= height) continue;
      int half = discHalfWidth[dy + discRadiusCells];
      if (half < 0) continue;
      int from = Math.max(0, cx - half);
      int to = Math.min(width - 1, cx + half);
      int row = gy * width;
      for (int gx = from; gx <= to; gx++) {
        double w = strength(row + gx);
        if (w > 0.0) {
          total += w / (1.0 + w);
        }
      }
    }
    return total / discCellCount;
  }

  // ---------------------------------------------------------------------------------------------
  // Effective vividness
  // ---------------------------------------------------------------------------------------------

  /**
   * Weighting between the vividness of the space and the agent's stable spatial ability:
   * {@code 0.7 * space + 0.3 * ability}. Neither weight has a source.
   */
  private static final double VIVIDNESS_WEIGHT = 0.7;

  private static final double MEMORY_ABILITY_WEIGHT = 0.3;

  /**
   * The vividness the agent's recognition thresholds follow: the vividness of the space blended
   * with its spatial ability, in [0, 1]. A function of its two arguments only.
   */
  public static double effectiveVividness(double spaceVividness, double spatialAbility) {
    double v = Math.max(0.0, Math.min(1.0, spaceVividness));
    double a = Math.max(0.0, Math.min(1.0, spatialAbility));
    return VIVIDNESS_WEIGHT * v + MEMORY_ABILITY_WEIGHT * a;
  }
}
