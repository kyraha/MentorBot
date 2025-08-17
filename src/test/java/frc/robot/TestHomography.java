package frc.robot;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Range;
import org.opencv.core.Scalar;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.sensors.Camera;

public class TestHomography {
    // Load OpenCV library before running tests
    static {
        OpenCvLoader.forceStaticLoad();
    }

    private MatOfPoint2f image = new MatOfPoint2f(new Point(571, 261.1), new Point(261.8, 240.1),
            new Point(764, 461), new Point(194, 342.5), new Point(138.9, 233.3), new Point(329.9, 336.7));
    private MatOfPoint2f world = new MatOfPoint2f(new Point(0.57, -0.18), new Point(0.65, -0.51), new Point(4, 0.88),
            new Point(4, -2.86), new Point(1, -1), new Point(2, -1));
    private static Mat world2Robot = new Mat(4, 4, CvType.CV_64F, Scalar.all(0));
    private static Mat robot2Camera = new Mat(4,4, CvType.CV_64F, Scalar.all(0));
    private static Mat wpi2Opencv = new Mat(4,4, CvType.CV_64F, Scalar.all(0));

    @BeforeAll
    static void setup() {
        // Initialize the transformation matrix from front-left corner of the robot to its center
        world2Robot.put(0, 3, -0.381);   // forward offset in x
        world2Robot.put(1, 3, -0.381);   // left offset in y
        world2Robot.put(2, 3, -0.206);   // height of the algae center
        world2Robot.put(0, 0, 1.0); // No rotation so it's identity
        world2Robot.put(1, 1, 1.0);
        world2Robot.put(2, 2, 1.0);
        world2Robot.put(3, 3, 1.0); // The homogeneous one

        // Initialize our homogenius robot to the camera transformation from Camera.robotToCamera
        robot2Camera.put(0,0, Camera.robotToCamera.toMatrix().getData());

        // To rotate from WPI to OpenCV coordinate system, roll Y down and then yaw X to the right
        Transform3d tWpi2Opencv = new Transform3d(
            new edu.wpi.first.math.geometry.Translation3d(0,0,0),
            new edu.wpi.first.math.geometry.Rotation3d(-Math.PI/2, 0, -Math.PI/2));
        wpi2Opencv.put(0, 0, tWpi2Opencv.toMatrix().getData());

        // System.out.println("World to Robot Matrix:\n" + world2Robot.dump());
        // System.out.println("Robot to Camera Matrix:\n" + robot2Camera.dump());
        // System.out.println("World to Camera Matrix:\n" + world2Robot.matMul(robot2Camera).dump());
        // System.out.println("WPI to OpenCV Matrix:\n" + opencv2wpi.dump());
    }

    @Test
    void testGeneralHomography() {
        MatOfPoint2f imagePoints = new MatOfPoint2f(new Point(400,240));
        MatOfPoint2f worldPoints = new MatOfPoint2f();
        Core.perspectiveTransform(imagePoints, worldPoints, Camera.homographyMatrix);
        System.out.println("Transformed Point: " + worldPoints.toList().get(0).toString());
    }

    @Test
    void testMats() {
        // Compare our "theoretical" projections to the actual measured points
        assert image.rows() == world.rows() : "Image and world must have the same number of points";
        for (int i=0; i < image.rows(); i++) {
            Point wP2d = world.toList().get(i);
            Point aP2d = image.toList().get(i);

            // Compose a homogeneous point in world coordinates
            Mat wPointH = new Mat(4, 1, CvType.CV_64F, new Scalar(0));
            wPointH.put(0, 0, new double[]{wP2d.x, wP2d.y, 0.0, 1.0});

            // Transform to camera coordinates and rotate into OpenCV coordinate system
            Mat cPointH = world2Robot.matMul(robot2Camera).matMul(wpi2Opencv).inv().matMul(wPointH);

            // Project the homogeneous point back to 3D coordinates with scaling by homogeneous dimension
            Mat cP3d = new Mat(cPointH, new Range(0, 3));
            Core.divide(cP3d, new Scalar(cPointH.get(3, 0)[0]), cP3d);

            // Project the 3D point to 2D image coordinates using the camera intrinsics
            Mat iP2d = Camera.cameraMatrix.matMul(cP3d);

            // Normalize the projected point from homogeneous coordinates
            Core.divide(iP2d, new Scalar(iP2d.get(2, 0)[0]), iP2d);

            // Convert to Point for easier handling
            Point iP2dPoint = new Point(iP2d.get(0, 0)[0], iP2d.get(1, 0)[0]);

            Point diffPoint = new Point(iP2dPoint.x - aP2d.x, iP2dPoint.y - aP2d.y);
            double error = Math.sqrt(diffPoint.dot(diffPoint));

            // Print the projected and actual points
            System.out.println("World point "+ i +": "+ wP2d.toString());
            System.out.println(" Projected to image:" + iP2dPoint.toString());
            System.out.println(" Actual image point:" + aP2d.toString() + " Error: " + error);
        }
    }
}
