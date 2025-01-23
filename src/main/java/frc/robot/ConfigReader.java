package frc.robot;

import java.io.File;
import java.io.FileReader;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonPrimitive;

import edu.wpi.first.wpilibj.Filesystem;

public class ConfigReader {
  final private JsonObject root;
  /**
  * Parses a JSON file with config items from the given file name.
  *
  * @param configName the name of the JSON config file in the "deploy/config/" folder
  * @return a JsonElement object with all the configuration items
  */
  public static JsonElement readConfig(String configName) {
    try (
      FileReader fileReader = new FileReader(new File(
        // Find the path to the config directory inside the deploy direcotry
        new File(Filesystem.getDeployDirectory(), "config"),
        // And then append the config file name to the path
        configName))
    ) {
      // Parse the entire file and return as a JSON element
      return JsonParser.parseReader(fileReader);
    }
    catch (Exception e) {
      throw new RuntimeException(String.format("Error reading config file: %s", configName), e);
    }
  }

  public ConfigReader(String configName) {
    root = readConfig(configName).getAsJsonObject();
  }

  public ConfigReader(JsonObject newRoot) {
    root = newRoot;
  }

  public JsonObject getRoot() {
    return root;
  }

  public ConfigReader getAsSubReader(String path) {
    String nodes[] = path.split("/");
    String leaf = nodes[nodes.length-1];
    JsonObject pointer = root;

    // Iterate through all but one. The last one is the primitive we want
    for (int i=0; i+1 < nodes.length; i++) {
      pointer = pointer.getAsJsonObject(nodes[i]);
    }
    return new ConfigReader(pointer.getAsJsonObject(leaf));
  }

  public JsonArray getAsJsonArray(String path) {
    String nodes[] = path.split("/");
    String leaf = nodes[nodes.length-1];
    JsonObject pointer = root;

    // Iterate through all but one. The last one is the primitive we want
    for (int i=0; i+1 < nodes.length; i++) {
      pointer = pointer.getAsJsonObject(nodes[i]);
    }
    return pointer.getAsJsonArray(leaf);
  }

  public JsonPrimitive getAsJsonPrimitive(String path) {
    String nodes[] = path.split("/");
    String leaf = nodes[nodes.length-1];
    JsonObject pointer = root;

    // Iterate through all but one. The last one is the primitive we want
    for (int i=0; i+1 < nodes.length; i++) {
      pointer = pointer.getAsJsonObject(nodes[i]);
    }
    return pointer.getAsJsonPrimitive(leaf);
  }

  public double getAsDouble(String path) {
    return getAsJsonPrimitive(path).getAsDouble();
  }

  public String getAsString(String path) {
    return getAsJsonPrimitive(path).getAsString();
  }

  public int getAsInt(String path) {
    return getAsJsonPrimitive(path).getAsInt();
  }
}
