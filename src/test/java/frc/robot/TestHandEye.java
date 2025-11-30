package frc.robot;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.Test;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

import frc.robot.sensors.HandEyeCalibration;

public class TestHandEye {
    // Load OpenCV library before running tests
    static {
        OpenCvLoader.forceStaticLoad();
    }

    /*
     * Data sampled from the robot sensors in the shop. Every two rows contain (every 4 double arrays):
     * - first row: Camera to target transform (cTt): {tx, ty, tz}, and {rotation matrix 3x3}
     * - second row: Odometry (base to gripper) transform (bTg): {tx, ty, tz}, and {rotation matrix 3x3}
     * The robot started at an arbitrary position and moved around while looking at a fixed target.
     * The camera was mounted rigidly on the robot. The target was static in the world (bTt is fixed but unknown).
     */
    static double[][] data = {
        {1.045542451462151, -0.5562181931493166, -0.1926964077866995}, // Vis Trans, time=32.426846
        {-0.8261555410102377, 0.4081575639585938, -0.38842557207458434, 0.2966814689805617, 0.9011953670939317, 0.31595413637420516, 0.4790063966395623, 0.14578859115795112, -0.8656203317081699}, // Vis Rotat
        {-1.0742493187638629, -0.04690463025733676, 0.0}, // Odo Trans
        {0.74535039143568, -0.6666729287939316, 0.0, 0.6666729287939316, 0.74535039143568, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {1.3461937782437605, -0.4938498313401532, -0.22124769347125706}, // Vis Trans, time=73.947199
        {-0.7820479721390023, 0.5984234569155982, -0.17404118905149984, 0.5525499911214886, 0.7949488476188749, 0.25048919533797387, 0.28825245286886536, 0.09972810979161326, -0.9523470100401823}, // Vis Rotat
        {-1.3557623916432193, -0.04571646858059377, 0.0}, // Odo Trans
        {0.7653509597071914, -0.6436131667976356, 0.0, 0.6436131667976356, 0.7653509597071914, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {1.5745488503762144, -0.40934678553820103, -0.23818118572404656}, // Vis Trans, time=79.348403
        {-0.7761694116331488, 0.6060959484234119, -0.17380663321569978, 0.5605559009535295, 0.7895118767579155, 0.2498961351529294, 0.28868343622596165, 0.0965334023171208, -0.9525456292933439}, // Vis Rotat
        {-1.6239398235183848, -0.03860779969895233, 0.0}, // Odo Trans
        {0.7816053135026134, -0.6237733032957417, 0.0, 0.6237733032957417, 0.7816053135026134, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {2.0022512473073624, -0.27873749909602213, -0.273835443531116}, // Vis Trans, time=84.447853
        {-0.7586245744643776, 0.6289240090710623, -0.170127439975816, 0.584421523611628, 0.7723011591905439, 0.24900281575196215, 0.28799346825614947, 0.08947351746145188, -0.9534433658665153}, // Vis Rotat
        {-2.0926379893387748, -0.03422973789163897, 0.0}, // Odo Trans
        {0.803490955335153, -0.5953169615378039, 0.0, 0.5953169615378039, 0.803490955335153, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {2.3167016014072104, -0.2055578409208842, -0.30695924500412053}, // Vis Trans, time=89.227595
        {-0.7395547817388639, 0.6526348645352702, -0.1647011183939307, 0.6098951976941931, 0.7532607450886055, 0.24622367420319302, 0.2847570414173162, 0.08164547447096243, -0.955116455654319}, // Vis Rotat
        {-2.4350195394593634, -0.03058031039563914, 0.0}, // Odo Trans
        {0.8198381296443488, -0.5725953555358758, 0.0, 0.5725953555358758, 0.8198381296443488, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {2.1124404458405293, 0.4990457439619032, -0.11787477286208714}, // Vis Trans, time=96.94651
        {-0.7957287977355176, 0.5840944047781258, -0.16015432170651736, 0.5424191522572501, 0.8049212220632647, 0.24058946264686393, 0.2694385713273323, 0.10457319244943919, -0.9573229881821594}, // Vis Rotat
        {-2.4553469043589264, 0.5099736603467903, 0.0}, // Odo Trans
        {0.7606717806148303, -0.6491366899014902, 0.0, 0.6491366899014902, 0.7606717806148303, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {2.041775234918125, 1.0075229049126884, 0.005455337972277485}, // Vis Trans, time=104.066371
        {-0.9056247118573204, 0.3866910044622289, -0.1741090128090383, 0.3428394311623947, 0.9092364698645596, 0.23611473123987697, 0.24960970676957062, 0.15413990053268545, -0.9559999400367045}, // Vis Rotat
        {-2.4524548113795284, 0.5021073865106614, 0.0}, // Odo Trans
        {0.5944284130851594, -0.8041485321239226, 0.0, 0.8041485321239226, 0.5944284130851594, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {1.9239223560097227, 0.56423214106049, -0.07399201895178154}, // Vis Trans, time=112.08752
        {-0.8738123067708454, 0.4584898767821552, -0.1619848308456643, 0.4164152356415232, 0.8775744202436295, 0.23761626303700714, 0.25109839517891985, 0.14017906342368555, -0.9577574985956674}, // Vis Rotat
        {-2.172030789683342, 0.4018534500507405, 0.0}, // Odo Trans
        {0.6554583036615036, -0.7552313633325778, 0.0, 0.7552313633325778, 0.6554583036615036, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {1.7310477451044213, 0.24322137032507718, -0.11704120088677872}, // Vis Trans, time=118.306705
        {-0.8844163232463769, 0.43688762762312644, -0.16412485492818396, 0.3935406883718828, 0.8871752706968413, 0.24092688861928827, 0.25086548939849346, 0.1484898646164873, -0.9565653487007735}, // Vis Rotat
        {-1.8302819941012647, 0.24348304788449632, 0.0}, // Odo Trans
        {0.6391615343505294, -0.7690725147908207, 0.0, 0.7690725147908207, 0.6391615343505294, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {1.4722463382391746, -0.14170880901524485, -0.16260697904456123}, // Vis Trans, time=124.786275
        {-0.8852015824233033, 0.426810374799232, -0.18507042561959783, 0.3784393359462924, 0.8920311328044926, 0.24711156815508017, 0.2705583624355552, 0.1487056221912026, -0.9511492051433983}, // Vis Rotat
        {-1.422838883168566, 0.09963849953287014, 0.0}, // Odo Trans
        {0.6400453513218245, -0.7683371318967489, 0.0, 0.7683371318967489, 0.6400453513218245, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {1.2260517990115107, -0.516036735747937, -0.20845413821085113}, // Vis Trans, time=131.366368
        {-0.8828212411103638, 0.45196320791158495, -0.12789024567427754, 0.4180654285290689, 0.8801895423583057, 0.2246946082839602, 0.21412135277258926, 0.14489868259354793, -0.966000216392052}, // Vis Rotat
        {-1.0257907322698696, -0.04221027389396913, 0.0}, // Odo Trans
        {0.6415913289578541, -0.76704665217058, 0.0, 0.76704665217058, 0.6415913289578541, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {0.7675129623977259, -0.17591622516407268, -0.06052048742414673}, // Vis Trans, time=154.967515
        {-0.9021805037455011, 0.3934600107594279, -0.17680372901809718, 0.3473698418171423, 0.9056945067032502, 0.24300545986353814, 0.2557430969888481, 0.15781850481089668, -0.9537760679959448}, // Vis Rotat
        {-0.8128345160087954, 0.5020361655931915, 0.0}, // Odo Trans
        {0.5744746906638664, -0.8185223453190847, 0.0, 0.8185223453190847, 0.5744746906638664, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {0.7629700209416437, 0.19459305322829507, 0.02234665735839969}, // Vis Trans, time=159.986533
        {-0.9255799499655102, 0.3377938074044605, -0.1708715889229158, 0.29416337120351177, 0.9259078768285504, 0.23698631747219268, 0.238263860591632, 0.16908562122795706, -0.9563683314654072}, // Vis Rotat
        {-1.0335017167299954, 0.7741738748860179, 0.0}, // Odo Trans
        {0.5520180079631001, -0.8338321886833411, 0.0, 0.8338321886833411, 0.5520180079631001, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
        {0.8006687724642814, 0.28591960188988963, 0.0366153506444965}, // Vis Trans, time=171.366308
        {-0.8400399847646303, 0.5178784430375979, -0.1616624329694249, 0.4753206532731291, 0.8461985267204607, 0.2408699440530668, 0.2615398642055281, 0.12549889088384505, -0.9569989173548104}, // Vis Rotat
        {-1.2619134825434564, 0.9628684651886916, 0.0}, // Odo Trans
        {0.7043218119028185, -0.7098808247008301, 0.0, 0.7098808247008301, 0.7043218119028185, 0.0, 0.0, 0.0, 1.0}, // Odo Rotat
    };

    /**
     * Helper function to create a 4x4 homogeneous transformation matrix from
     * a rotation matrix and translation vector.
     */
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
     * \[ cTt = gTc^{-1} * bTg^{-1} * bTt \]
     * where:
     * - gTc is the robot-to-camera transform (fixed, predefined for this test, but what we
     *   want to find in general)
     * - bTg is the robot movement (odometry measurement)
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
    void testHandEyeModeled() {
        // Choose calibration method
        int calib_method = Calib3d.CALIB_HAND_EYE_HORAUD;
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
        Mat r_world2target1 = Mat.eye(3, 3, CvType.CV_64F);
        Mat t_world2target1 = Mat.zeros(3, 1, CvType.CV_64F);
        Mat r_world2target2 = Mat.eye(3, 3, CvType.CV_64F);
        Mat t_world2target2 = Mat.zeros(3, 1, CvType.CV_64F);

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
        t_world2target1.put(0, 0, 2.0);
        // No rotation between world and target

        // And the target2 is half a meter above the first target
        t_world2target2.put(0, 0, 2.0);
        t_world2target2.put(2, 0, 0.5);

        // Compute the fixed transforms that will be the same for all "measurements"
        Mat bTt1 = makeHomogeneous(r_world2target1, t_world2target1);
        Mat bTt2 = makeHomogeneous(r_world2target2, t_world2target2);
        Mat gTc = makeHomogeneous(r_robot2cam, t_robot2cam);
        System.out.println("bTt 1 and 2 (world to target lower and upper):\n" + TestHomography.matToString(bTt1) + TestHomography.matToString(bTt2));
        System.out.println("gTc (robot to camera, predefined):\n" + TestHomography.matToString(gTc));

        // Simulate a series of odometry and visual measurements
        // In real life, these would be collected from the robot odometry and PhotonPoseEstimator
        ArrayList<Mat> ls_odo_r = new ArrayList<>(); // Rotation part of odometry measurements
        ArrayList<Mat> ls_odo_t = new ArrayList<>(); // Translation part of odometry measurements
        ArrayList<Mat> ls_vis_r = new ArrayList<>(); // Rotation part of visual measurements
        ArrayList<Mat> ls_vis_t = new ArrayList<>(); // Translation part of visual measurements
        final int numMeasurements = 40;
        for (int i = 0; i < numMeasurements; i++) {
            // To simulate movement, let's say the robot moves in some direction and rotates a bit
            double x = ((double)i / numMeasurements) * 2.0; // Move 2m forward over the measurements
            double y = ((double)i*i / numMeasurements/numMeasurements) * -1.0; // Move 1m sideways in a curve
            double z1 = 0.0;           // For lower target, the height is 0.5m
            double z2 = -0.5;           // For upper target, the height is 1.0m
            double theta = Math.toRadians(i); // Rotate a degrees each step

            // Create new rotation matrices and translation vectors to store the simulated data
            var r_odo = Mat.eye(3, 3, CvType.CV_64F);
            var t_odo1 = Mat.zeros(3, 1, CvType.CV_64F);
            var t_odo2 = Mat.zeros(3, 1, CvType.CV_64F);
            var r_visual1 = new Mat();
            var t_visual1 = new Mat();
            var r_visual2 = new Mat();
            var t_visual2 = new Mat();

            // Odometry measurement, just what we assumed above about how the robot moves
            t_odo1.put(0, 0, x);
            t_odo1.put(1, 0, y);
            t_odo1.put(2, 0, z1);
            t_odo2.put(0, 0, x);
            t_odo2.put(1, 0, y);
            t_odo2.put(2, 0, z2);
            // We'll use the same odometry rotation for both targets
            r_odo.put(0, 0, Math.cos(theta));
            r_odo.put(0, 1, -Math.sin(theta));
            r_odo.put(1, 0, Math.sin(theta));
            r_odo.put(1, 1, Math.cos(theta));
            Mat bTg = makeHomogeneous(r_odo, t_odo1);

            // Visual measurement needs to be modeled, let's use transformations:
            // \[ cTt = gTc^{-1} * bTg^{-1} * bTt \] -- see the description of this test for details
            Mat cTt1 = gTc.inv().matMul(bTg.inv()).matMul(bTt1);
            Mat cTt2 = gTc.inv().matMul(bTg.inv()).matMul(bTt2);

            // Extract rotation and translation from cTt into separate matrices
            cTt1.submat(0, 3, 0, 3).copyTo(r_visual1);
            cTt1.submat(0, 3, 3, 4).copyTo(t_visual1);
            cTt2.submat(0, 3, 0, 3).copyTo(r_visual2);
            cTt2.submat(0, 3, 3, 4).copyTo(t_visual2);

            // Add some noise to the visual measurement
            // Add noise to translation in all 3 axes
            t_visual1.put(0, 0, t_visual1.get(0, 0)[0] + random.nextGaussian() * noiseLevel);
            t_visual1.put(1, 0, t_visual1.get(1, 0)[0] + random.nextGaussian() * noiseLevel);
            t_visual1.put(2, 0, t_visual1.get(2, 0)[0] + random.nextGaussian() * noiseLevel);
            t_visual2.put(0, 0, t_visual2.get(0, 0)[0] + random.nextGaussian() * noiseLevel);
            t_visual2.put(1, 0, t_visual2.get(1, 0)[0] + random.nextGaussian() * noiseLevel);
            t_visual2.put(2, 0, t_visual2.get(2, 0)[0] + random.nextGaussian() * noiseLevel);
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
            r_visual1 = r_noise.matMul(r_visual1);
            r_visual2 = r_noise.matMul(r_visual2);

            // Add the "measurements" to the lists
            ls_odo_r.add(r_odo);
            ls_odo_t.add(t_odo1);
            ls_vis_r.add(r_visual1);
            ls_vis_t.add(t_visual1);
            // The upper visuals go with the same odometry rotation
            ls_odo_r.add(r_odo);
            ls_odo_t.add(t_odo2);
            ls_vis_r.add(r_visual2);
            ls_vis_t.add(t_visual2);

            // For debugging, print out the simulated measurements
            // System.out.println(HandEyeCalibration.flatStringMat(t_visual1, "Vis Trans, time=") + i);
            // System.out.println(HandEyeCalibration.flatStringMat(r_visual1, "Vis Rotat"));
            // System.out.println(HandEyeCalibration.flatStringMat(t_odo1, "Odo Trans"));
            // System.out.println(HandEyeCalibration.flatStringMat(r_odo, "Odo Rotat"));
        }

        // Prepare empty matrices to receive the calibration result
        Mat r_calibrated = new Mat();
        Mat t_calibrated = new Mat();

        // Now perform hand-eye calibration to find the camera-to-robot transform
        Calib3d.calibrateHandEye(ls_odo_r, ls_odo_t, ls_vis_r, ls_vis_t, r_calibrated, t_calibrated, calib_method);
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

    @Test
    void testHandEyeReal() {
        // In this test, we use real collected data from the robot
        ArrayList<Mat> r_odos = new ArrayList<>(); // Rotation part of odometry measurements
        ArrayList<Mat> t_odos = new ArrayList<>(); // Translation part of odometry measurements
        ArrayList<Mat> r_visuals = new ArrayList<>(); // Rotation part of visual measurements
        ArrayList<Mat> t_visuals = new ArrayList<>(); // Translation part of visual measurements

        System.out.println("Loaded " + (data.length / 4) + " measurement pairs from real robot data.");
        for (int i = 0; i < data.length / 4; i++) {
            // Visual measurement +0 and +1
            Mat t_vis = Mat.zeros(3, 1, CvType.CV_64F);
            t_vis.put(0, 0, data[i * 4 + 0]);
            t_visuals.add(t_vis);
            Mat r_vis = Mat.zeros(3, 3, CvType.CV_64F);
            r_vis.put(0, 0, data[i * 4 + 1]);
            r_visuals.add(r_vis);

            // Odometry measurement +2 and +3
            Mat t_odo = Mat.zeros(3, 1, CvType.CV_64F);
            t_odo.put(0, 0, data[i * 4 + 2]);
            t_odos.add(t_odo);
            Mat r_odo = Mat.zeros(3, 3, CvType.CV_64F);
            r_odo.put(0, 0, data[i * 4 + 3]);
            r_odos.add(r_odo);
        }

        // Prepare empty matrices to receive the calibration result
        Mat r_calibrated = new Mat();
        Mat t_calibrated = new Mat();

        for (int method = 0; method < 5; method++) {
            // Now perform hand-eye calibration to find the camera-to-robot transform
            try {
                Calib3d.calibrateHandEye(r_odos, t_odos, r_visuals, t_visuals, r_calibrated, t_calibrated, method);
                System.out.println("Calibrated with method " + method + ":");
                System.out.println("Calibrated Rotation:\n" + r_calibrated.dump());
                System.out.println("Calibrated Translation:\n" + t_calibrated.dump());
                double[] r_buffer = new double[9];
                r_calibrated.get(0, 0, r_buffer);
                double[] t_buffer = new double[3];
                t_calibrated.get(0, 0, t_buffer);
                Matrix<N3,N3> r_matrix = new Matrix<N3,N3>(N3.instance, N3.instance, r_buffer);
                Rotation3d r_result = new Rotation3d(r_matrix);
                System.out.println("robotToCamera Rotation3d in Axis-Angle:\n" + r_result.getAxis() + r_result.getAngle());
                System.out.println("robotToCamera Rotation3d in Roll,Pitch,Yaw: " + r_result.getX() + "," + r_result.getY() + "," + r_result.getZ());

                Mat robotToCamera = makeHomogeneous(r_calibrated, t_calibrated);
                System.out.println("robotToCamera estimated with method " +method+ ":\n" + TestHomography.matToString(robotToCamera));
            } catch (Exception e) {
                // e.printStackTrace();
                System.out.println("calibrateHandEye failed with method " + method + ": " + e.getMessage());
            }
        }
    }

    /**
     * This function is just to help export odometry data from robot logs
     * It is not a real test.
     * So we just print out the data in the required format.
     */
    @Test
    void exportOdometryData() {
        for (int i=0; i < data.length; i+=4) {
            // Odometry data is in the 3rd and 4th row of each 4-row block
            double[] odoT = data[i+2];
            double[] odoR = data[i+3];
            Matrix<N3,N3> r_matrix = new Matrix<N3,N3>(N3.instance, N3.instance, odoR);
            Rotation3d r_rotation = new Rotation3d(r_matrix);
            // All rotations should be about Z axis only for odometry on flat surface
            // So the angle extracted from the rotation matrix is all we need

            double x = odoT[0];
            double y = odoT[1];
            double theta = r_rotation.toRotation2d().getRadians();
            var axis = r_rotation.getAxis();
            // System.out.println("Axis: " + axis + " Angle: " + Math.toDegrees(theta));

            System.out.println(x + "\t" + y + "\t" + theta);
        }
    }
}
