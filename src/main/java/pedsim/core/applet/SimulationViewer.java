package pedsim.core.applet;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.geom.Path2D;
import java.util.ArrayList;
import java.util.List;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.Timer;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;

import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import sim.util.geo.MasonGeometry;

public class SimulationViewer extends JFrame {
  private static final long serialVersionUID = 1L;

  private MapPanel mapPanel;
  private JLabel statusLabel;
  private Timer updateTimer;

  private boolean running = true;

  public SimulationViewer() {
    setTitle("PedSimCity - Live Simulation");
    setSize(1000, 720);
    setLayout(new BorderLayout());

    mapPanel = new MapPanel();
    add(mapPanel, BorderLayout.CENTER);

    statusLabel = new JLabel(" Waiting for simulation to start...");
    statusLabel.setOpaque(true);
    statusLabel.setBackground(Color.WHITE);
    statusLabel.setForeground(new Color(50, 50, 50));
    statusLabel.setFont(new Font("SansSerif", Font.PLAIN, 14));
    statusLabel.setBorder(BorderFactory.createEmptyBorder(5, 10, 5, 10));
    add(statusLabel, BorderLayout.SOUTH);

    // Refresh at 30 FPS (~33ms) to capture smooth snapshots
    updateTimer = new Timer(33, e -> updateDisplay());
    updateTimer.start();
  }

  private void updateDisplay() {
    PedSimCity state = PedSimCity.currentInstance;
    if (state != null && state.schedule != null) {
      if (mapPanel.paths == null && PedSimCity.MBR != null && !PedSimCity.roads.getGeometries().isEmpty()) {
        mapPanel.regeneratePaths();
      }
      
      long currentStep = state.schedule.getSteps();
      if (currentStep > mapPanel.lastStepRendered) {
        mapPanel.lastStepRendered = currentStep;
        
        // Take a snapshot for the flow visualization
        for (Agent agent : state.agentsWalking) {
          if (agent.getLocation() != null && agent.getLocation().getGeometry() != null) {
            Coordinate c = agent.getLocation().getGeometry().getCoordinate();
            mapPanel.flowTrail.add(new Coordinate(c));
          }
        }
        
        // Cap the memory of the trail
        while (mapPanel.flowTrail.size() > mapPanel.MAX_FLOW_POINTS) {
          mapPanel.flowTrail.poll();
        }
      }
      
      mapPanel.repaint();
      updateStatusBar(state);
    }
  }

  private void updateStatusBar(PedSimCity state) {
    if (state.schedule == null) return;
    long steps = state.schedule.getSteps();
    
    // 1 step = 20 minutes in PedSimCity
    long totalMins = steps * 20;
    long days = (totalMins / (24 * 60)) + 1;
    long hours = (totalMins % (24 * 60)) / 60;
    long mins = totalMins % 60;

    int total = state.agentsList.size();
    int walking = state.agentsWalking.size();
    int atHome = state.agentsAtHome.size();
    int atDestination = total - walking - atHome;

    statusLabel.setText(String.format(" Day %d | %02d:%02d | %d agents · %d walking · %d at home · %d at destination",
        days, hours, mins, total, walking, atHome, Math.max(0, atDestination)));
  }

  class MapPanel extends JPanel {
    private static final long serialVersionUID = 1L;
    
    List<Path2D.Double> paths;
    Projector projector;

    private double zoomFactor = 4.0; // Started zoomed in
    private int panX = 0;
    private int panY = 0;
    private int dragOriginX;
    private int dragOriginY;
    
    long lastStepRendered = -1;
    final int MAX_FLOW_POINTS = 60000;
    final java.util.Queue<Coordinate> flowTrail = new java.util.concurrent.ConcurrentLinkedQueue<>();

    public MapPanel() {
      setBackground(new Color(15, 20, 35)); // Navy dark background
      addComponentListener(new ComponentAdapter() {
        @Override
        public void componentResized(ComponentEvent e) {
          regeneratePaths();
        }
      });

      java.awt.event.MouseAdapter ma = new java.awt.event.MouseAdapter() {
        @Override
        public void mousePressed(java.awt.event.MouseEvent e) {
          dragOriginX = e.getX();
          dragOriginY = e.getY();
        }

        @Override
        public void mouseDragged(java.awt.event.MouseEvent e) {
          panX += e.getX() - dragOriginX;
          panY += e.getY() - dragOriginY;
          dragOriginX = e.getX();
          dragOriginY = e.getY();
          regeneratePaths();
          repaint();
        }

        @Override
        public void mouseWheelMoved(java.awt.event.MouseWheelEvent e) {
          double oldZoom = zoomFactor;
          if (e.getWheelRotation() < 0) {
            zoomFactor *= 1.25;
          } else {
            zoomFactor /= 1.25;
          }
          
          double scaleChange = zoomFactor / oldZoom;
          panX = (int) (e.getX() - (e.getX() - panX) * scaleChange);
          panY = (int) (e.getY() - (e.getY() - panY) * scaleChange);
          
          regeneratePaths();
          repaint();
        }
      };
      addMouseListener(ma);
      addMouseMotionListener(ma);
      addMouseWheelListener(ma);
    }

    public void regeneratePaths() {
      if (PedSimCity.MBR == null) return;
      int w = getWidth();
      int h = getHeight();
      if (w <= 0 || h <= 0) return;

      Envelope env = PedSimCity.MBR;
      double baseScale = Math.min((double)w / env.getWidth(), (double)h / env.getHeight()) * 0.95;
      
      double scale = baseScale * zoomFactor;

      int xBaseOffset = (int) ((w - (env.getWidth() * baseScale)) / 2);
      int yBaseOffset = (int) ((h - (env.getHeight() * baseScale)) / 2);
      
      int cx = w / 2;
      int cy = h / 2;
      
      int xOffset = (int) (cx - (cx - xBaseOffset) * zoomFactor) + panX;
      int yOffset = (int) (cy - (cy - yBaseOffset) * zoomFactor) + panY;
      
      projector = new Projector(env.getMinX(), env.getMaxY(), scale, xOffset, yOffset);
      
      paths = new ArrayList<>();
      for (Object obj : PedSimCity.roads.getGeometries()) {
        MasonGeometry mg = (MasonGeometry) obj;
        Coordinate[] coords = mg.getGeometry().getCoordinates();
        if (coords.length < 2) continue;
        Path2D.Double path = new Path2D.Double();
        path.moveTo(projector.x(coords[0].x), projector.y(coords[0].y));
        for (int i = 1; i < coords.length; i++) {
           path.lineTo(projector.x(coords[i].x), projector.y(coords[i].y));
        }
        paths.add(path);
      }
    }

    @Override
    protected void paintComponent(Graphics g) {
      super.paintComponent(g);
      PedSimCity state = PedSimCity.currentInstance;
      if (state == null || projector == null || paths == null) return;

      Graphics2D g2 = (Graphics2D) g;
      g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

      // Draw roads
      g2.setColor(new Color(255, 255, 255, 30));
      for (Path2D.Double p : paths) {
        g2.draw(p);
      }
      
      // Draw flow heatmap trail
      g2.setColor(new Color(0, 200, 255, 20)); // Glowing electric blue
      for (Coordinate c : flowTrail) {
        int px = projector.x(c.x);
        int py = projector.y(c.y);
        g2.fillRect(px - 1, py - 1, 3, 3);
      }

      // Draw active agents
      for (Agent agent : state.agentsList) {
        Coordinate coord = null;
        if (agent.getLocation() != null && agent.getLocation().getGeometry() != null) {
             coord = agent.getLocation().getGeometry().getCoordinate();
        }
        if (coord == null) continue;

        int px = projector.x(coord.x);
        int py = projector.y(coord.y);

        Color mainColor;
        int size;

        if (state.agentsWalking.contains(agent)) {
          mainColor = new Color(240, 175, 50); // Amber
          size = 8;
          // Glow
          g2.setColor(new Color(240, 175, 50, 60));
          g2.fillOval(px - size, py - size, size * 2, size * 2);
        } else if (state.agentsAtHome.contains(agent)) {
          mainColor = new Color(74, 130, 208); // Steel blue
          size = 5;
        } else {
          mainColor = new Color(90, 200, 120); // Sage green
          size = 6;
        }

        g2.setColor(mainColor);
        g2.fillOval(px - size/2, py - size/2, size, size);
      }
    }
  }

  static class Projector {
    double minX, maxY, scale;
    int xOffset, yOffset;

    Projector(double minX, double maxY, double scale, int xOffset, int yOffset) {
      this.minX = minX;
      this.maxY = maxY;
      this.scale = scale;
      this.xOffset = xOffset;
      this.yOffset = yOffset;
    }

    int x(double geoX) {
      return (int) ((geoX - minX) * scale) + xOffset;
    }

    int y(double geoY) {
      return (int) ((maxY - geoY) * scale) + yOffset; // y is flipped
    }
  }
}
