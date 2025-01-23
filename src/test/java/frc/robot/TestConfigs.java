package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.Drivetrain.SwerveModule;

/** Add your docs here. */
public class TestConfigs {
    private static String configName = "DrivetrainConfigPrototypERR.json";
    @Test
    void simpleTest() {
        var drivetrainConfig = ConfigReader.readConfig(configName);
        var swerveModulesConfig = drivetrainConfig.getAsJsonObject().getAsJsonArray("swerveModules");
        var nModules = swerveModulesConfig.size();
        assertEquals(4, nModules, "Should be 4 swerve modules");
        for(int i=0; i < nModules; i++) {
            var moduleConf = new ConfigReader(swerveModulesConfig.get(i).getAsJsonObject());
            assertTrue(moduleConf.getRoot().has("name"), "Swerve module must have a name");
            var moduleName = moduleConf.getAsJsonPrimitive("name").getAsString();
            assertTrue(moduleName.length() > 5, "Name must be longer than 5");
            var oneModule = new SwerveModule(moduleConf);
            assertEquals(moduleName, oneModule.getName(), "Module name should get assigned");
            System.out.println("Steer '" + moduleName + "': " + oneModule.getSteerSettings());
            System.out.println("Drive '" + moduleName + "': " + oneModule.getDriveSettings());
        }
    }

}
