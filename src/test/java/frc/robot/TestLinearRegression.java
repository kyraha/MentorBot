package frc.robot;

import java.util.ArrayList;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.junit.jupiter.api.Test;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import edu.wpi.first.math.Vector;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

public class TestLinearRegression {
    // Load OpenCV library before running tests
    static {
        OpenCvLoader.forceStaticLoad();
    }

    /**
     * This test demonstrates how find camera rotation using linear regression by making multiple
     * measurements of 3D points and then derive the rotation needed to align the camera's "down"
     * direction with the horizontal plane normal.
     * 
     * The plane equation is of the form: ax + by + cz + d = 0 (in camera coordinates)
     * For our purposes, we fix c = -1, so the equation becomes: ax + by - z + d = 0
     * Rearranging gives: z = d + ax + by
     * 
     * We can use ordinary least squares regression to find the coefficients a, b, and d.
     * 
     * Once we have the plane normal vector (a, b, -1), we can compute the rotation
     * needed to align the camera's "down" vector (0, 0, -1) with the plane normal.
     */
    @Test
    void testRollPitch() {
        // Pretend we collected a bunch of sample data points (x, y, z)
        // In reality, just drive around the same AprilTag and record a few getBestCameraToTarget()
        double[] z = {
            -0.0533,
            -0.343,
            -0.822
        };
        double[][] xy = {
            {0.487, -0.1498},
            {3.45, 1.39},
            {2.94, -1.295}
        };

        // Create a regression model and fit the data
        OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();
        regression.newSampleData(z, xy);
        System.out.println( // Just to see how good the fit is
            String.format("Standard Error: % .4f; R^2: % .4f",
            regression.estimateRegressionStandardError(),
            regression.calculateRSquared()));

        double[] coefficients = regression.estimateRegressionParameters();
        double d = coefficients[0];
        double a = coefficients[1];
        double b = coefficients[2];
        double c = -1.0; // c is fixed to -1 (why? read above)
        System.out.println(String.format("Plane equation: %.3fx %+.3fy %+.3fz %+.3f = 0", a, b, c, d));

        // The plane normal vector from the camera POV is (a, b, c)
        Vector<N3> planeNormal = VecBuilder.fill(a, b, c);

        // Camera's own "down" direction is simply (0, 0, -1)
        Vector<N3> cameraNormal = VecBuilder.fill(0, 0, -1);

        // Camera is rotated as if plane would be rotated to match the camera normal
        // i.e, the rotation direction is from true "down" (plane normal) to camera "down" (camera normal)
        Rotation3d rotation1 = new Rotation3d(planeNormal, cameraNormal);
        System.out.println(String.format("If right side up. Roll = %.3f, Pitch = %.3f, Yaw = %.3f\n  %s",
            rotation1.getX(),
            rotation1.getY(),
            rotation1.getZ(),
            rotation1.getQuaternion()));

        // Alternatively, if the camera is upside down, the camera normal would point "up"
        cameraNormal.set(2, 0, 1); // Set Z component to +1
        Rotation3d rotation2 = new Rotation3d(planeNormal, cameraNormal);
        System.out.println(String.format("If upside down. Roll = %.3f, Pitch = %.3f, Yaw = %.3f\n  %s",
            rotation2.getX(),
            rotation2.getY(),
            rotation2.getZ(),
            rotation2.getQuaternion()));

        // However, yaw here is fantom and must be adjusted further (to be relative to what?)
    }

    Mat makeHomogeneous(Mat r, Mat t) {
        Mat h = Mat.eye(4, 4, CvType.CV_64F);
        r.copyTo(h.submat(0, 3, 0, 3));
        t.copyTo(h.submat(0, 3, 3, 4));
        return h;
    }

    @Test
    void testHandEye() {
        // Fix robot2camera and world2target transforms to some constant values
        // For simulation purposes and for later verification
        Mat r_robot2cam = Mat.eye(3, 3, CvType.CV_64F);
        Mat t_robot2cam = Mat.zeros(3, 1, CvType.CV_64F);
        Mat r_world2target = Mat.eye(3, 3, CvType.CV_64F);
        Mat t_world2target = Mat.zeros(3, 1, CvType.CV_64F);

        // Let's say the camera is rotated 20 degrees looking down
        double angle = Math.toRadians(20);
        r_robot2cam.put(0, 0, Math.cos(angle));
        r_robot2cam.put(0, 2, Math.sin(angle));
        r_robot2cam.put(2, 0, -Math.sin(angle));
        r_robot2cam.put(2, 2, Math.cos(angle));
        // And the camera is 0.5m above the robot center, and 0.3m forward, 0.1m to the left
        t_robot2cam.put(2, 0, 0.5);
        t_robot2cam.put(0, 0, 0.3);
        t_robot2cam.put(1, 0, 0.1);

        // Let's say the target is 2m in front of the robot at the start
        t_world2target.put(0, 0, 2.0);
        // And the target is 1m above the ground
        t_world2target.put(2, 0, 1.0);
        // No rotation between world and target

        // Compute the fixed transforms
        Mat bTt = makeHomogeneous(r_world2target, t_world2target);
        Mat gTc = makeHomogeneous(r_robot2cam, t_robot2cam);
        System.out.println("bTt (world to target):\n" + bTt.dump());
        System.out.println("gTc (robot to camera):\n" + gTc.dump());

        // Simulate a series of odometry and visual measurements
        ArrayList<Mat> r_odos = new ArrayList<>();
        ArrayList<Mat> t_odos = new ArrayList<>();
        ArrayList<Mat> r_visuals = new ArrayList<>();
        ArrayList<Mat> t_visuals = new ArrayList<>();
        for (int i = 0; i < 10; i++) {
            // Let's simulate odometry and visual measurements
            double x = i * 0.1;     // Move +0.1m in X each step
            double y = i * -0.05;   // Move -0.05m in Y each step
            double z = 0;           // No movement in Z
            double theta = Math.toRadians(i * 5); // Rotate +5 degrees each step

            // Create new rotation matrices and translation vectors to store the simulated data
            var r_odo = Mat.eye(3, 3, CvType.CV_64F);
            var t_odo = Mat.zeros(3, 1, CvType.CV_64F);
            var r_visual = Mat.eye(3, 3, CvType.CV_64F);
            var t_visual = Mat.zeros(3, 1, CvType.CV_64F);

            // Odometry measurement, direct movement
            t_odo.put(0, 0, x); // Move 0.1m in X each step
            t_odo.put(1, 0, y); // Move -0.05m in Y each step
            t_odo.put(2, 0, z); // No movement in Z
            r_odo.put(0, 0, Math.cos(theta));
            r_odo.put(0, 1, -Math.sin(theta));
            r_odo.put(1, 0, Math.sin(theta));
            r_odo.put(1, 1, Math.cos(theta));

            // Visual measurement, cTt = bTg^-1 * gTc^-1 * bTc
            Mat bTg = makeHomogeneous(r_odo, t_odo);
            // System.out.println("Move: " + i + " bTg:\n" + bTg.dump());
            Mat cTt_h = gTc.inv().matMul(bTg.inv()).matMul(bTt);

            double noiseLevel = 0.9; // add some noise
            double noiseAngle = Math.toRadians((Math.random() - 0.5) * 0.5);

            // Extract rotation and translation from cTt_h
            cTt_h.submat(0, 3, 0, 3).copyTo(r_visual);
            cTt_h.submat(0, 3, 3, 4).copyTo(t_visual);
            // System.out.println("Cam to target:\n" + cTt_h.dump());

            // Add the "measurements" to the lists
            r_odos.add(r_odo);
            t_odos.add(t_odo);
            r_visuals.add(r_visual);
            t_visuals.add(t_visual);
        }

        // Prepare output matrices
        Mat r_cam2robot = new Mat();
        Mat t_cam2robot = new Mat();

        // Now perform hand-eye calibration to find the camera-to-robot transform
        Calib3d.calibrateHandEye(r_odos, t_odos, r_visuals, t_visuals, r_cam2robot, t_cam2robot);
        System.out.println("Calibrated Rotation:\n" + r_cam2robot.dump());
        System.out.println("Calibrated Translation:\n" + t_cam2robot.dump());
    }
}
