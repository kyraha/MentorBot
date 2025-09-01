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

    private static Mat world2Robot = Mat.eye(4, 4, CvType.CV_64F);
    private static Mat robot2Camera = new Mat(4,4, CvType.CV_64F, Scalar.all(0));
    private static Mat wpi2Opencv = new Mat(4,4, CvType.CV_64F, Scalar.all(0));

    @BeforeAll
    static void setup() {
        // Initialize the transformation matrix from front-left corner of the robot to its center
        world2Robot.put(0, 3, -0.381);   // forward offset in x
        world2Robot.put(1, 3, -0.381);   // left offset in y
        world2Robot.put(2, 3, -0.206);   // height of the algae center

        // Initialize our homogenius robot to the camera transformation from Camera.robotToCamera
        robot2Camera.put(0,0, Camera.robotToCamera.toMatrix().getData());

        // To rotate from WPI to OpenCV coordinate system, roll Y down and then yaw X to the right
        Transform3d tWpi2Opencv = new Transform3d(
            new edu.wpi.first.math.geometry.Translation3d(0,0,0),
            new edu.wpi.first.math.geometry.Rotation3d(-Math.PI/2, 0, -Math.PI/2));
        wpi2Opencv.put(0, 0, tWpi2Opencv.toMatrix().getData());
    }

    String matToString(Mat m) {
        StringBuilder sb = new StringBuilder();
        if (m.empty()) {
            return "[]";
        }
        else if (m.rows() == 1) {
            sb.append("[");
            for (int j = 0; j < m.cols(); j++) {
                sb.append(String.format("% .4f", m.get(0, j)[0]));
                if (j < m.cols() - 1) {
                    sb.append(", ");
                }
            }
            sb.append("]");
            return sb.toString();
        }
        else if (m.cols() == 1) {
            sb.append("[");
            for (int i = 0; i < m.rows(); i++) {
                sb.append(String.format("% .4f", m.get(i, 0)[0]));
                if (i < m.rows() - 1) {
                    sb.append(", ");
                }
            }
            sb.append("]");
            return sb.toString();
        }
        for (int i = 0; i < m.rows(); i++) {
            sb.append("[");
            for (int j = 0; j < m.cols(); j++) {
                sb.append(String.format("% .4f", m.get(i, j)[0]));
                if (j < m.cols() - 1) {
                    sb.append(", ");
                }
            }
            sb.append("]\n");
        }
        return sb.toString();
    }

    Mat reduceHomo(Mat h) {
        // Reduce a homogeneous matrix so that the bottom-right value is 1.0
        Mat r = h.clone().rowRange(0, h.rows()-1);
        for(int i=0; i < r.cols(); i++) {
            double scale = h.get(h.rows()-1, i)[0];
            if (scale == 0.0) scale = 1.0;
            Core.divide(r.colRange(i, i+1), new Scalar(scale), r.colRange(i, i+1));
        }
        return r;
    }

    @Test
    void testGeneralHomography() {
        MatOfPoint2f imagePoints = new MatOfPoint2f(new Point(400,240));
        MatOfPoint2f worldPoints = new MatOfPoint2f();
        Core.perspectiveTransform(imagePoints, worldPoints, Camera.homographyMatrix);
        System.out.println("Transformed Point: " + worldPoints.toList().get(0).toString());
    }

    @Test
    void testArtificialHomography() {
        Mat cameraInv = Camera.cameraMatrix.inv();
        System.out.println("Camera Matrix Inverse:\n" + matToString(cameraInv));
        // MatOfPoint2f imagePoints = new MatOfPoint2f(new Point(571, 261.1)); //406,218));
        // MatOfPoint2f worldPoints = new MatOfPoint2f();
        // Mat iPoint3d = new Mat(3, 1, CvType.CV_64F, new Scalar(0));
        // iPoint3d.put(0, 0, imagePoints.toList().get(0).x);
        // iPoint3d.put(1, 0, imagePoints.toList().get(0).y);
        // iPoint3d.put(2, 0, 1.0); // Homogeneous coordinate

        // Mat cPoint3d = Camera.cameraMatrix.inv().matMul(iPoint3d);
        // System.out.println("Camera Matrix Inverse Applied: " + cPoint3d.dump());

        // Mat cPoint4d = new Mat(4, 1, CvType.CV_64F, new Scalar(0));
        // cPoint4d.put(0, 0, cPoint3d.get(0, 0)[0]);
        // cPoint4d.put(1, 0, cPoint3d.get(1, 0)[0]);
        // cPoint4d.put(2, 0, cPoint3d.get(2, 0)[0]);
        // cPoint4d.put(3, 0, 1.0); // Homogeneous coordinate

        // // Transform to world coordinates
        // Mat wPoint4d = wpi2Opencv.matMul(cPoint4d); //robot2Camera).matMul(world2Robot).matMul(cPoint4d);
        // System.out.println("World Point camera rotated: " + wPoint4d.dump());
        // wPoint4d = robot2Camera.matMul(wPoint4d);
        // System.out.println("World Point robot to camera shifted: " + wPoint4d.dump());
        // wPoint4d = world2Robot.matMul(wPoint4d);
        // System.out.println("World Point shifted to robot corner: " + wPoint4d.dump());

        // Same chain of transformations but with one matrix
        Mat comboMat = world2Robot.matMul(robot2Camera).matMul(wpi2Opencv);
        System.out.println("Combo Matrix:\n" + matToString(comboMat));
        Mat camToWorldTranslation = Mat.eye(4, 4, CvType.CV_64F);
        // projectionMat.put(0, 0, comboMat.get(2, 3));
        // projectionMat.put(1, 1, comboMat.get(2, 3));
        camToWorldTranslation.put(0, 3, -comboMat.get(0, 3)[0]);
        camToWorldTranslation.put(1, 3, -comboMat.get(1, 3)[0]);
        // projectionMat.put(2, 2, 1.0/comboMat.get(2, 3)[0]);
        System.out.println("Cam to World Matrix:\n" + matToString(camToWorldTranslation));
        System.out.println("World to cam Matrix:\n" + matToString(camToWorldTranslation.inv()));

        double f = comboMat.get(2, 3)[0];
        System.out.println("Focal length (from combo matrix): " + f);

        // Compare our "theoretical" projections to the actual measured points
        assert image.rows() == world.rows() : "Image and world must have the same number of points";
        for (int i=0; i < image.rows(); i++) {
            Point wPoint2d = world.toList().get(i);
            Point aPoint2d = image.toList().get(i);
            System.out.println("\nPoint " + i + " Image: " + aPoint2d + " World: " + wPoint2d);

            Mat aPoint3d = Mat.ones(3, 1, CvType.CV_64F);
            aPoint3d.put(0, 0, aPoint2d.x);
            aPoint3d.put(1, 0, aPoint2d.y);
            Mat inCamCoord3d = cameraInv.matMul(aPoint3d);

            Mat homoInCam4d = Mat.ones(4, 1, CvType.CV_64F);
            homoInCam4d.put(0, 0, inCamCoord3d.get(0, 0)[0]);
            homoInCam4d.put(1, 0, inCamCoord3d.get(1, 0)[0]);
            homoInCam4d.put(2, 0, inCamCoord3d.get(2, 0)[0]);
            System.out.println("Point " + i + " in Camera coords: " + matToString(homoInCam4d));

            Mat homoCamInWpi4d = wpi2Opencv.matMul(homoInCam4d);
            System.out.println("Point " + i + " in WPI coords: " + matToString(homoCamInWpi4d));

            Mat homoInRobot4d = robot2Camera.matMul(homoCamInWpi4d);
            System.out.println("Point " + i + " in Robot coords: " + matToString(homoInRobot4d));

            Mat homoInWorld4d = world2Robot.matMul(homoInRobot4d);
            System.out.println("Point " + i + " in World coords: " + matToString(homoInWorld4d));

            // All above in one step
            // Mat wPoint4d = comboMat.matMul(homoInCam4d);
            // Mat reduced = reduceHomo(wPoint4d);

            Mat homoShifted = camToWorldTranslation.matMul(homoInWorld4d);
            double z = homoInWorld4d.get(2, 0)[0];
            double scale = f / (f - z);
            Mat homoProjected = new Mat(4, 1, CvType.CV_64F);
            Core.multiply(homoShifted, new Scalar(scale), homoProjected);
            Mat homoUnshifted = camToWorldTranslation.inv().matMul(homoProjected);
            System.out.println("Point " + i + " projected: " + matToString(homoUnshifted));
            System.out.println("Scale: " + scale);
        }

        // Test the artificial homography matrix
        Mat artificialHomography = new Mat(3, 3, CvType.CV_64F);
        artificialHomography.put(0, 0, 1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0);
        // Core.perspectiveTransform(imagePoints, worldPoints, artificialHomography);
        // System.out.println("Transformed Point with Artificial Homography: " + worldPoints.toList().get(0).toString());
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
