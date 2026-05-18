package pedsim.cityimage.applet;

import java.awt.Color;
import java.awt.Graphics;
import java.util.List;
import javax.swing.JPanel;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import sim.util.geo.MasonGeometry;

public class Display extends JPanel {
  /**
   * 
   */
  private static final long serialVersionUID = 1L;

  private double minX, maxX, minY, maxY;
  private List<Agent> agentList;
  private int panelWidth;
  private int panelHeight;
  // maximum allowed dimension
  private int maxResolution = 900;
  private double scaleFactor = 1.0;

  public Display(List<Agent> agentList) {

    this.agentList = agentList;
    defineSize();
  }

  private void defineSize() {
    Envelope envelope = PedSimCity.roads.getMBR(); // Get the bounding box
    minX = envelope.getMinX();
    maxX = envelope.getMaxX();
    minY = envelope.getMinY();
    maxY = envelope.getMaxY();

    // Calculate dimensions of the bounding box
    double width = maxX - minX;
    double height = maxY - minY;

    // Calculate the larger dimension of the bounding box
    double maxDimension = Math.max(width, height);

    System.out.println(width + " " + height);

    // Check if rescaling is needed
    if (maxDimension > maxResolution) {
      // Calculate the scale factor to bring the largest dimension down to
      // maxResolution
      scaleFactor = maxDimension / maxResolution;

      // Rescale the width and height
      panelWidth = (int) (width / scaleFactor);
      panelHeight = (int) (height / scaleFactor);
    } else {
      // If no scaling is needed, keep the original dimensions
      panelWidth = (int) width;
      panelHeight = (int) height;
    }

    System.out.println(panelWidth + " " + panelHeight);

    // Print the resulting dimensions and set the size
    this.setSize(panelWidth, panelHeight);
    PedSimCityImageApplet.frame.setSize((int) (panelWidth * 1.05), (int) (panelHeight * 1.05));
  }

  private int mapToPanelX(double x) {
    return (int) ((x - minX) / (maxX - minX) * panelWidth);
  }

  private int mapToPanelY(double y) {
    return (int) ((maxY - y) / (maxY - minY) * panelHeight); // Invert Y for correct display
  }

  @Override
  protected void paintComponent(Graphics graphic) {

    super.paintComponent(graphic);

    // Draw background
    graphic.setColor(Color.BLACK);
    graphic.fillRect(0, 0, getWidth(), getHeight());
    System.out.println("in paint " + getWidth() + " " + getHeight());

    renderRoads(graphic);
    renderLandmarks(graphic);
    renderAgents(graphic);
  }

  private void renderRoads(Graphics graphic) {

    // Draw roads
    graphic.setColor(new Color(64, 64, 64));
    for (MasonGeometry road : PedSimCity.roads.getGeometries()) {
      Geometry geometry = road.getGeometry();
      if (geometry == null || geometry.getCoordinates().length < 2)
        continue;

      Coordinate[] coords = geometry.getCoordinates();
      for (int i = 0; i < coords.length - 1; i++) {
        int x1 = mapToPanelX(coords[i].x);
        int y1 = mapToPanelY(coords[i].y);
        int x2 = mapToPanelX(coords[i + 1].x);
        int y2 = mapToPanelY(coords[i + 1].y);
        graphic.drawLine(x1, y1, x2, y2);
      }
    }
  }

  private void renderLandmarks(Graphics graphic) {
    // Draw landmarks

    graphic.setColor(new Color(128, 128, 0));

    for (MasonGeometry landmark : PedSimCity.buildings.getGeometries()) { // Assuming 'landmarks'
                                                                          // exists
      Geometry geometry = landmark.getGeometry();

      Coordinate[] coords = geometry.getCoordinates();
      int[] xPoints = new int[coords.length];
      int[] yPoints = new int[coords.length];

      for (int i = 0; i < coords.length; i++) {
        xPoints[i] = mapToPanelX(coords[i].x);
        yPoints[i] = mapToPanelY(coords[i].y);
      }

      graphic.drawPolygon(xPoints, yPoints, coords.length);
      graphic.fillPolygon(xPoints, yPoints, coords.length); // Optionally fill
    }
  }

  private void renderAgents(Graphics graphic) {

    // Draw agents
    graphic.setColor(Color.RED);

    for (Agent agent : agentList) {
      Coordinate point = agent.getGeometry().getGeometry().getCoordinate();
      int x = mapToPanelX(point.x);
      int y = mapToPanelY(point.y);
      graphic.fillOval(x - 3, y - 3, 6, 6); // Draw agent as a small circle

    }
  }

  // private void renderAgents(Graphics graphic) {
  // // Define colors for each group
  // Color[] groupColors = {
  // Color.GRAY, // NULLGROUP
  // Color.BLUE, // POPULATION
  // Color.RED, // GROUP1
  // Color.GREEN, // GROUP2
  // Color.YELLOW, // GROUP3
  // Color.CYAN, // GROUP4
  // Color.MAGENTA, // GROUP5
  // Color.ORANGE // GROUP6
  // };
  //
  // // Iterate through agents and assign colors based on their group
  // for (Agent agent : agentList) {
  // // Get the group index
  // int groupIndex = agent.group.ordinal(); // Convert enum to index
  // if (groupIndex < 0 || groupIndex >= groupColors.length) {
  // groupIndex = 0; // Default to NULLGROUP color if index is out of bounds
  // }
  //
  // // Set the color based on the group
  // graphic.setColor(groupColors[groupIndex]);
  //
  // // Draw the agent
  // Coordinate point = agent.getGeometry().getGeometry().getCoordinate();
  // int x = mapToPanelX(point.x);
  // int y = mapToPanelY(point.y);
  // graphic.fillOval(x - 3, y - 3, 6, 6); // Draw agent as a small circle
  // }
  // }

}
