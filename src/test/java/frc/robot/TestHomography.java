package frc.robot;

import org.junit.jupiter.api.Test;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import edu.wpi.first.cscore.OpenCvLoader;
import frc.robot.sensors.Camera;

public class TestHomography {
    // Load OpenCV library before running tests
    static {
        OpenCvLoader.forceStaticLoad();
    }


    // @BeforeAll
    // static void setup() {

    // }

    @Test
    void testGeneralHomography() {
        MatOfPoint2f imagePoints = new MatOfPoint2f(new Point(400,240));
        MatOfPoint2f worldPoints = new MatOfPoint2f();
        Core.perspectiveTransform(imagePoints, worldPoints, Camera.homographyMatrix);
        System.out.println("Transformed Point: " + worldPoints.toList().get(0).toString());
    }

    @Test
    void testMats() {
        System.out.println("R2C Translation: " + Camera.robotToCamera.getTranslation());
        System.out.println("R2C Rotation: " + Camera.robotToCamera.getRotation().toMatrix().toString());
    }
}
