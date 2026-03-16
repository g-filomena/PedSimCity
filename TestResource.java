import sim.graph.Graph;
import java.lang.reflect.Method;

public class TestResource {
    public static void main(String[] args) {
        try {
            Class<?> clazz = Graph.class;
            System.out.println("Methods in Graph:");
            for (Method method : clazz.getMethods()) {
                System.out.println(method.getName());
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
