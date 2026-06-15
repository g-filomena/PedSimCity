import java.time.LocalDateTime;

public class TestTime {
  public static void main(String[] args) {
    double STEP_DURATION = 1200; // TimePars.STEP_DURATION
    double totalSteps = 1.0;
    long totalMinutes = (long) (totalSteps * (STEP_DURATION / 60));
    long totalDays = totalMinutes / (24 * 60);
    long remainingMinutes = totalMinutes % (24 * 60);

    long hours = remainingMinutes / 60;
    long minutes = remainingMinutes % 60;

    LocalDateTime dt =
        LocalDateTime.of(1970, 1, 1, 0, 0)
            .plusDays(totalDays)
            .plusHours(hours)
            .plusMinutes(minutes);

    int day = (int) (totalSteps * STEP_DURATION / 86_400.0) + 1;
    String formatted = String.format("Day%d %02d:%02d", day, dt.getHour(), dt.getMinute());
    System.out.println("Step " + totalSteps + " = " + formatted);

    totalSteps = 2.0;
    totalMinutes = (long) (totalSteps * (STEP_DURATION / 60));
    totalDays = totalMinutes / (24 * 60);
    remainingMinutes = totalMinutes % (24 * 60);
    hours = remainingMinutes / 60;
    minutes = remainingMinutes % 60;
    dt =
        LocalDateTime.of(1970, 1, 1, 0, 0)
            .plusDays(totalDays)
            .plusHours(hours)
            .plusMinutes(minutes);
    day = (int) (totalSteps * STEP_DURATION / 86_400.0) + 1;
    formatted = String.format("Day%d %02d:%02d", day, dt.getHour(), dt.getMinute());
    System.out.println("Step " + totalSteps + " = " + formatted);
  }
}
