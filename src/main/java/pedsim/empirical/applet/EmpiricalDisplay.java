package pedsim.empirical.applet;

import java.awt.Color;
import java.awt.Graphics;
import java.util.Collection;
import java.util.Collections;
import javax.swing.JPanel;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import sim.util.geo.MasonGeometry;

/** Minimal visualisation panel for empirical simulations. */
public class EmpiricalDisplay extends JPanel {

  private static final long serialVersionUID = 1L;

  private static final int MAX_RESOLUTION = 900;

  private double minX = 0.0;
  private double maxX = 1.0;
  private double minY = 0.0;
  private double maxY = 1.0;

  private int panelWidth = MAX_RESOLUTION;
  private int panelHeight = MAX_RESOLUTION;

  private Collection<? extends Agent> agents = Collections.emptyList();

  public EmpiricalDisplay(Collection<? extends Agent> agents) {
    this.agents = agents;
    defineSize();
  }

  public void setAgents(Collection<? extends Agent> agents) {
    this.agents = agents;
  }

  private void defineSize() {
    if (PedSimCity.roads == null || PedSimCity.roads.getGeometries().isEmpty()) {
      setSize(panelWidth, panelHeight);
      setPreferredSize(getSize());
      return;
    }

    Envelope envelope = PedSimCity.roads.getMBR();

    if (envelope == null || envelope.isNull()) {
      setSize(panelWidth, panelHeight);
      setPreferredSize(getSize());
      return;
    }

    minX = envelope.getMinX();
    maxX = envelope.getMaxX();
    minY = envelope.getMinY();
    maxY = envelope.getMaxY();

    double width = Math.max(1.0, maxX - minX);
    double height = Math.max(1.0, maxY - minY);
    double maxDimension = Math.max(width, height);

    double scaleFactor = maxDimension / MAX_RESOLUTION;

    panelWidth = Math.max(400, (int) (width / scaleFactor));
    panelHeight = Math.max(400, (int) (height / scaleFactor));

    setSize(panelWidth, panelHeight);
    setPreferredSize(getSize());
  }

  private int mapToPanelX(double x) {
    return (int) ((x - minX) / Math.max(1.0, maxX - minX) * panelWidth);
  }

  private int mapToPanelY(double y) {
    return (int) ((maxY - y) / Math.max(1.0, maxY - minY) * panelHeight);
  }

  @Override
  protected void paintComponent(Graphics graphics) {
    super.paintComponent(graphics);

    graphics.setColor(Color.BLACK);
    graphics.fillRect(0, 0, getWidth(), getHeight());

    renderRoads(graphics);
    renderBuildings(graphics);
    renderAgents(graphics);
  }

  private void renderRoads(Graphics graphics) {
    graphics.setColor(new Color(64, 64, 64));

    for (MasonGeometry road : PedSimCity.roads.getGeometries()) {
      Geometry geometry = road.getGeometry();

      if (geometry == null || geometry.getCoordinates().length < 2) {
        continue;
      }

      Coordinate[] coordinates = geometry.getCoordinates();

      for (int i = 0; i < coordinates.length - 1; i++) {
        int x1 = mapToPanelX(coordinates[i].x);
        int y1 = mapToPanelY(coordinates[i].y);
        int x2 = mapToPanelX(coordinates[i + 1].x);
        int y2 = mapToPanelY(coordinates[i + 1].y);

        graphics.drawLine(x1, y1, x2, y2);
      }
    }
  }

  private void renderBuildings(Graphics graphics) {
    graphics.setColor(new Color(128, 128, 0));

    for (MasonGeometry building : PedSimCity.buildings.getGeometries()) {
      Geometry geometry = building.getGeometry();

      if (geometry == null || geometry.getCoordinates().length < 3) {
        continue;
      }

      Coordinate[] coordinates = geometry.getCoordinates();
      int[] xPoints = new int[coordinates.length];
      int[] yPoints = new int[coordinates.length];

      for (int i = 0; i < coordinates.length; i++) {
        xPoints[i] = mapToPanelX(coordinates[i].x);
        yPoints[i] = mapToPanelY(coordinates[i].y);
      }

      graphics.drawPolygon(xPoints, yPoints, coordinates.length);
    }
  }

  private void renderAgents(Graphics graphics) {
    graphics.setColor(Color.RED);

    for (Agent agent : agents) {
      if (agent == null || agent.getLocation() == null || agent.getLocation().getGeometry() == null) {
        continue;
      }

      Coordinate point = agent.getLocation().getGeometry().getCoordinate();

      int x = mapToPanelX(point.x);
      int y = mapToPanelY(point.y);

      graphics.fillOval(x - 3, y - 3, 6, 6);
    }
  }
}