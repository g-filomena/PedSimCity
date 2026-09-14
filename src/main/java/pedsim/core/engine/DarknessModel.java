package pedsim.core.engine;

/**
 * When a module considers an hour of a simulated day to be dark. Supplied through
 * {@link FlowHandler#setDarknessModel}; with none set the volume exports carry no light/dark split.
 */
@FunctionalInterface
public interface DarknessModel {

  /**
   * @param clockHour hour of day, 0-23
   * @param day the simulated day, counting from 1
   * @return whether that hour of that day is dark
   */
  boolean isDark(int clockHour, int day);
}
