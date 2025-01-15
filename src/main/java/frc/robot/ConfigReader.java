package frc.robot;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

import com.google.gson.JsonElement;
import com.google.gson.JsonParser;

import edu.wpi.first.wpilibj.Filesystem;

public class ConfigReader {
  /**
  * Parses a JSON file with config items from the given file name.
  *
  * @param configName the name of the JSON config file in the "deploy/config/" folder
  * @return a JsonElement object with all the configuration items
  */
  public static JsonElement readConfig(String configName) {
    try (BufferedReader br =
        new BufferedReader(
          new FileReader(
            new File(
              new File(
                Filesystem.getDeployDirectory(), "config"), configName)))) {
      return JsonParser.parseReader(br);
    }
    catch (Exception e) {
      throw new RuntimeException(String.format("Error reading config file: %s", configName), e);
    }
  }
}
