package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** Add your docs here. */
public class TestConfigs {
    @Test
    void simpleTest() {
        var drivetrainConfig = ConfigReader.readConfig("DrivetrainConfig.json");
        var swerveModulesConfig = drivetrainConfig.getAsJsonObject().getAsJsonArray("swerveModules");
        var nModules = swerveModulesConfig.size();
        assertEquals(4, nModules, "Should be 4 swerve modules");
        for(int i=0; i < nModules; i++) {
            var module = swerveModulesConfig.get(i).getAsJsonObject();
            assertTrue(module.has("name"), "Swerve module must have a name");
            assertTrue(module.getAsJsonPrimitive("name").getAsString().length() > 5, "Name must be longer than 5");
        }
    }

}
