package frc.robot;

import org.junit.jupiter.api.Test;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import edu.wpi.first.cscore.OpenCvLoader;
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
        // Transformation from left front corner of the robot to its center
        Mat world2Robot = new Mat(4, 4, CvType.CV_64F, new Scalar(0));
        world2Robot.put(0, 3, -0.381);   // forward offset in x
        world2Robot.put(1, 3, -0.381);   // left offset in y
        world2Robot.put(2, 3, -0.206);   // height of the algae center
        world2Robot.put(0, 0, 1.0); // No rotation so it's identity
        world2Robot.put(1, 1, 1.0);
        world2Robot.put(2, 2, 1.0);
        world2Robot.put(3, 3, 1.0); // The homogeneous one
        System.out.println("World to Robot Matrix:\n" + world2Robot.dump());

        // Transformation from the robot to the camera is inverse of the robotToCamera
        Mat robot2Camera = new Mat(4,4, CvType.CV_64F);
        robot2Camera.put(0,0, Camera.robotToCamera.toMatrix().getData());
        System.out.println("Robot to Camera Matrix:\n" + robot2Camera.dump());

        // Let's compare our projections to the actual measured points
        assert image.rows() == world.rows() : "Image and world must have the same number of points";
        for (int i=0; i < image.rows(); i++) {
            Point wP2d = world.toList().get(i);
            Point aP2d = image.toList().get(i);
            System.out.println("World point "+ i +": "+ wP2d.toString());

            // Compose a homogeneous point in world coordinates
            Mat wPointH = new Mat(4, 1, CvType.CV_64F, new Scalar(0));
            wPointH.put(0, 0, new double[]{wP2d.x, wP2d.y, 0.0, 1.0});

            // Transform to camera coordinates (WPI coordinate system)
            Mat cPointH = robot2Camera.inv().matMul(world2Robot.inv().matMul(wPointH));

            // Populate a 3D point in camera coordinates (OpenCV coordinate system)
            Mat cP3d = new Mat(3, 1, CvType.CV_64F);
            cP3d.put(0, 0, new double[]{
                -cPointH.get(1, 0)[0],      // camera x = negative robot y (to right)
                -cPointH.get(2, 0)[0],      // camera y = negative robot z (down)
                 cPointH.get(0, 0)[0]});    // camera z = robot x (forward)

            // Project the 3D point to 2D image coordinates using the camera intrinsics
            Mat iP2d = Camera.cameraMatrix.matMul(cP3d);
            // Normalize the projected point from homogeneous coordinates
            Core.divide(iP2d, new Scalar(iP2d.get(2, 0)[0]), iP2d);

            // Print the projected and actual points
            Point iP2dPoint = new Point(iP2d.get(0, 0)[0], iP2d.get(1, 0)[0]);
            System.out.println("Projected to image:" + iP2dPoint.toString());
            System.out.println("Actual image point:" + aP2d.toString());
        }
    }
}
