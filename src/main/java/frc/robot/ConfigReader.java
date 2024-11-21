package frc.robot;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.File;
import org.json.JSONObject;

import edu.wpi.first.wpilibj.Filesystem;

public class ConfigReader {
  /**
  * Builds a JSON with config items from the given file name.
  *
  * @param configName the path/name of the JSON config file in the "deploy" folder
  * @return a JSONObject with all the configuration items
  */
  public static JSONObject readConfig(String configName) {
    try (BufferedReader br =
        new BufferedReader(
            new FileReader(
                new File(
                    Filesystem.getDeployDirectory(), configName)))) {
      StringBuilder fileContentBuilder = new StringBuilder();
      String line;
      while ((line = br.readLine()) != null) {
        fileContentBuilder.append(line);
      }
      return new JSONObject(fileContentBuilder.toString());
    } catch (Exception e) {
      throw new RuntimeException(String.format("Error reading config file: %s", configName), e);
    }
  }
}
