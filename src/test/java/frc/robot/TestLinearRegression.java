package frc.robot;

import java.util.ArrayList;
import java.util.Random;

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

    /**
     * This test demonstrates how to perform hand-eye calibration using OpenCV's calibrateHandEye()
     * function. The goal is to find the transformation from the camera frame to the robot frame
     * given a series of robot movements and corresponding visual measurements of a fixed target.
     * The measurements can be taken, e.g. from robot's odometry and AprilTag detections.
     * 
     * We simulate a robot with a camera mounted on it, looking at a fixed target (like an AprilTag).
     * The robot moves around, and we record:
     * - The robot's movement (odometry) as a series of rotation and translation pairs (bTg)
     * - The camera's observed transform to the target (visual measurement) as a series of rotation
     *   and translation pairs (cTt)
     * We assume the target's position in the world is fixed and known (bTt).
     * 
     * To simulate the vision measurements (cTt) the relationship is defined as:
     * cTt = gTc^-1 * bTg^-1 * bTt
     * where:
     * - gTc is the robot-to-camera transform (fixed, predefined for this test, but what we
     *   want to find in general)
     * - bTg is the robot movement (odometry)
     * - bTt is the world-to-target transform (fixed, predefined for this test)
     * 
     * Naming of the matrices comes from the OpenCV: Calib3d documentation:
     * https://docs.opencv.org/4.10.0/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b
     * 
     * By collecting multiple pairs of (bTg, cTt), we can solve for gTc
     * 
     * gTc stands for "grip to camera", or for our case it'll be `Transform3d robotToCamera`
     * which is required by PhotonPoseEstimator.
     */
    @Test
    void testHandEye() {
        // Random number generator for simulating noise
        Random random = new Random();
        // Noise levels to simulate real-world inaccuracies
        double noiseLevel = 0.0254; // in meters
        double noiseAngle = 0.0175; // in radians, e.g. 0.017 ~= 1 degree

        // Fix robot2camera and world2target transforms to some arbitrary constant values
        // This is needed for simulation purposes and then for later verification
        // In real life, these would be unknown and what we want to find
        Mat r_robot2cam = Mat.eye(3, 3, CvType.CV_64F); // Eye in opencv-ese is I - identity matrix
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
        // No rotation between world and target

        // Compute the fixed transforms that will be the same for all "measurements"
        Mat bTt = makeHomogeneous(r_world2target, t_world2target);
        Mat gTc = makeHomogeneous(r_robot2cam, t_robot2cam);
        System.out.println("bTt (world to target, predefined):\n" + TestHomography.matToString(bTt));
        System.out.println("gTc (robot to camera, predefined):\n" + TestHomography.matToString(gTc));

        // Simulate a series of odometry and visual measurements
        // In real life, these would be collected from the robot odometry and PhotonPoseEstimator
        ArrayList<Mat> r_odos = new ArrayList<>(); // Rotation part of odometry measurements
        ArrayList<Mat> t_odos = new ArrayList<>(); // Translation part of odometry measurements
        ArrayList<Mat> r_visuals = new ArrayList<>(); // Rotation part of visual measurements
        ArrayList<Mat> t_visuals = new ArrayList<>(); // Translation part of visual measurements
        for (int i = 0; i < 40; i++) {
            // To simulate movement, let's say the robot moves in some direction and rotates a bit
            double x = i * 0.02;     // Move +2cm in X each step
            double y = i * -0.02;   // Move -1cm in Y each step
            double z = 0;           // No movement in Z
            double theta = Math.toRadians(i); // Rotate a degrees each step

            // Create new rotation matrices and translation vectors to store the simulated data
            var r_odo = Mat.eye(3, 3, CvType.CV_64F);
            var t_odo = Mat.zeros(3, 1, CvType.CV_64F);
            var r_visual = Mat.eye(3, 3, CvType.CV_64F);
            var t_visual = Mat.zeros(3, 1, CvType.CV_64F);

            // Odometry measurement, just what we assumed above about how the robot moves
            t_odo.put(0, 0, x);
            t_odo.put(1, 0, y);
            t_odo.put(2, 0, z);
            r_odo.put(0, 0, Math.cos(theta));
            r_odo.put(0, 1, -Math.sin(theta));
            r_odo.put(1, 0, Math.sin(theta));
            r_odo.put(1, 1, Math.cos(theta));

            // Visual measurement needs to be modeled, let's use transformations:
            // cTt = bTg^-1 * gTc^-1 * bTt -- see the description of this test for details
            Mat bTg = makeHomogeneous(r_odo, t_odo);
            Mat cTt = gTc.inv().matMul(bTg.inv()).matMul(bTt);

            // Extract rotation and translation from cTt into separate matrices
            cTt.submat(0, 3, 0, 3).copyTo(r_visual);
            cTt.submat(0, 3, 3, 4).copyTo(t_visual);

            // Add some noise to the visual measurement
            // Add noise to translation in all 3 axes
            t_visual.put(0, 0, t_visual.get(0, 0)[0] + random.nextGaussian() * noiseLevel);
            t_visual.put(1, 0, t_visual.get(1, 0)[0] + random.nextGaussian() * noiseLevel);
            t_visual.put(2, 0, t_visual.get(2, 0)[0] + random.nextGaussian() * noiseLevel);
            // Add noise to rotation (small rotation in random direction)
            // Create a small rotation vector with random angles
            Mat v_noise = Mat.zeros(3, 1, CvType.CV_64F);
            v_noise.put(0, 0, random.nextGaussian() * noiseAngle);
            v_noise.put(1, 0, random.nextGaussian() * noiseAngle);
            v_noise.put(2, 0, random.nextGaussian() * noiseAngle);
            // Convert rotation vector to rotation matrix
            Mat r_noise = new Mat();
            Calib3d.Rodrigues(v_noise, r_noise);
            // Apply the noise rotation to the visual rotation
            r_visual = r_noise.matMul(r_visual);

            // Add the "measurements" to the lists
            r_odos.add(r_odo);
            t_odos.add(t_odo);
            r_visuals.add(r_visual);
            t_visuals.add(t_visual);
        }

        // Prepare empty matrices to receive the calibration result
        Mat r_calibrated = new Mat();
        Mat t_calibrated = new Mat();

        // Now perform hand-eye calibration to find the camera-to-robot transform
        Calib3d.calibrateHandEye(r_odos, t_odos, r_visuals, t_visuals, r_calibrated, t_calibrated);
        // System.out.println("Calibrated Rotation:\n" + r_cam2robot.dump());
        // System.out.println("Calibrated Translation:\n" + t_cam2robot.dump());
        Mat robotToCamera = makeHomogeneous(r_calibrated, t_calibrated);
        System.out.println("robotToCamera estimated with calibrateHandEye:\n" + TestHomography.matToString(robotToCamera));
        // Verify the result against the predefined robotToCamera (gTc) by inverting it
        System.out.println(
            "Difference (should be close to [identity|zero]):\n" + 
            TestHomography.matToString(robotToCamera.matMul(gTc.inv())));

        /* Note:
         * Since we simulated driving on a flat surface, which what is going to be in real life,
         * the offset of the camera in Z axis (height) cannot be determined. So the
         * calibrateHandEye() is leaving it at zero, which is acceptable since we don't
         * really care about the Z coordinate on a flat FRC field. And we can't measure it
         * with the robot's odometry anyway.
         */
    }
}
