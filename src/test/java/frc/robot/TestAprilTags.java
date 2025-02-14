package frc.robot;

import java.nio.file.FileSystems;
import java.nio.file.Path;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;

public class TestAprilTags {
    private static String customFieldName = "2025-ERRshop-field.json";
    private static AprilTagFieldLayout aprilTagFieldLayout;

    @BeforeAll
    static void setup() {
        try {
            Path pathToLayout = FileSystems.getDefault().getPath(
                Filesystem.getDeployDirectory().toString(),
                "fields",
                customFieldName);

            // The field from AprilTagField JSON file
            aprilTagFieldLayout = new AprilTagFieldLayout(pathToLayout);
        }
        catch (Exception e) {
            throw new RuntimeException(String.format("Error reading filed file: %s", customFieldName), e);
        }
    }

    @Test
    void testAprilTagPose() {
        for (int i=1; i<=2; i++) {
            var pose1 = aprilTagFieldLayout.getTagPose(i);
            System.out.println("Pose of ID=1: " + pose1.get().toString());
            System.out.println("Rotation axis: "+ pose1.get().getRotation().getAxis().toString());
        }
    }

}
