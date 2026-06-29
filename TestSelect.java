import java.util.*;

public class TestSelect {
  public static void main(String[] args) {
    int agentsSize = 20000;
    int nrAgents = 19999;
    Random random = new Random();
    Set<Integer> selectedAgents = new HashSet<>();
    long start = System.currentTimeMillis();
    long iter = 0;
    while (selectedAgents.size() < Math.min(nrAgents, agentsSize)) {
      int weightedIndex = (int) (Math.pow(random.nextDouble(), 1.5) * agentsSize);
      selectedAgents.add(weightedIndex);
      iter++;
    }
    System.out.println("Done in " + (System.currentTimeMillis() - start) + "ms. Iters: " + iter);
  }
}
