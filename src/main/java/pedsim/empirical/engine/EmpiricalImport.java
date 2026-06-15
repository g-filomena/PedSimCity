package pedsim.empirical.engine;

import java.io.InputStream;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import pedsim.core.engine.Import;
import pedsim.core.parameters.Pars;
import pedsim.empirical.agent.EmpiricalAgentsGroup;
import pedsim.empirical.agent.EmpiricalGroup;
import pedsim.empirical.parameters.EmpiricalPars;

/**
 * Importer for the empirical ABM.
 *
 * It delegates base GIS/network import to core.Import and then loads empirical
 * group calibration from {cityName}/{cityName}_clusters.csv.
 */
public class EmpiricalImport extends Import {

  @Override
  public void importFiles() throws Exception {
    super.importFiles();
    readEmpiricalGroups();
  }

  protected void readEmpiricalGroups() throws Exception {
    String resourceName = Pars.cityName + "/" + Pars.cityName + "_clusters.csv";
    URL fileUrl = CLASSLOADER.getResource(resourceName);

    if (fileUrl == null) {
      throw new IllegalStateException("Empirical groups resource not found: " + resourceName);
    }

    String content;
    try (InputStream inputStream = fileUrl.openStream()) {
      content = new String(inputStream.readAllBytes(), StandardCharsets.UTF_8);
    }

    EmpiricalPars.empiricalGroups.clear();

    for (String line : normaliseClusterCsv(content).split("\\R")) {
      String trimmed = line.trim();

      if (trimmed.isEmpty() || trimmed.startsWith(",")) {
        continue;
      }

      String[] attributes = trimmed.split("\\s*,\\s*");

      if (attributes.length == 0 || attributes[0] == null || attributes[0].isBlank()) {
        continue;
      }

      String groupNameToken = attributes[0].trim().replace("\uFEFF", "");

      if ("group".equalsIgnoreCase(groupNameToken)
          || "groupName".equalsIgnoreCase(groupNameToken)
          || "cluster".equalsIgnoreCase(groupNameToken)) {
        continue;
      }

      EmpiricalGroup groupName;
      try {
        groupName = EmpiricalGroup.valueOf(groupNameToken);
      } catch (IllegalArgumentException exception) {
        throw new IllegalStateException(
            "Unknown empirical group in CSV: " + groupNameToken, exception);
      }

      EmpiricalAgentsGroup group = new EmpiricalAgentsGroup();
      group.setGroup(groupName, attributes);

      EmpiricalPars.empiricalGroups.add(group);
    }

    if (EmpiricalPars.empiricalGroups.isEmpty()) {
      throw new IllegalStateException("No empirical groups loaded from " + resourceName);
    }
  }

  /**
   * Some repository text files currently appear collapsed into one physical line.
   * This normaliser makes the clusters CSV readable even if rows are separated by
   * spaces instead of newlines.
   */
  private static String normaliseClusterCsv(String content) {
    return content
        .replace("\r\n", "\n")
        .replace('\r', '\n')
        .replaceAll("\\s+(GROUP\\d|POPULATION|NULLGROUP),", "\n$1,");
  }
}
