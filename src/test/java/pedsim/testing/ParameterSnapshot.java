package pedsim.testing;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.LinkedHashMap;
import java.util.Map;

/** Restores mutable static parameters after a test, including the explicit-parameter key set. */
public final class ParameterSnapshot implements AutoCloseable {
  private final Map<Field, Object> values = new LinkedHashMap<>();

  public ParameterSnapshot(Class<?>... classes) throws IllegalAccessException {
    for (Class<?> cls : classes) {
      for (Field field : cls.getDeclaredFields()) {
        if (!Modifier.isStatic(field.getModifiers()) || Modifier.isFinal(field.getModifiers()))
          continue;
        field.setAccessible(true);
        values.put(field, field.get(null));
      }
    }
  }

  @Override
  public void close() throws IllegalAccessException {
    for (var entry : values.entrySet()) entry.getKey().set(null, entry.getValue());
  }
}
