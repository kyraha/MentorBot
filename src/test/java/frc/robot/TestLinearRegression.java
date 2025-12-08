package frc.robot;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.sensors.HandEyeCalibration;

import static frc.robot.TestHandEye.data;

public class TestLinearRegression {
    // Load OpenCV library before running tests
    static {
        OpenCvLoader.forceStaticLoad();
    }

    /**
     * This test demonstrates how to find camera rotation using linear regression by making multiple
     * measurements of 3D points and then derive the rotation needed to align the camera's "down"
     * direction with the horizontal plane normal. Where the plane is what all the measured points belong to.
     * For this experiment, we assume that the target is located lower than the camera.
     */
    @Test
    void testRollPitch() {
        // Pretend we collected a bunch of sample data points (x, y, z)
        // In reality, just drive around the same AprilTag and record a few getBestCameraToTarget()
        // double[][] tTcTranslationsDoubleArray = {
        //     {0.487, -0.1498, -0.0533},
        //     {3.45, 1.39, -0.343},
        //     {2.94, -1.295, -0.822}
        // };

        // // Prepare data for roll and pitch calculation
        // ArrayList<Mat> t_visuals = new ArrayList<>();
        // for (double[] tTcTranslation : tTcTranslationsDoubleArray) {
        //     Mat mat = new Mat(3, 1, CvType.CV_64F);
        //     mat.put(0, 0, tTcTranslation);
        //     t_visuals.add(mat);
        // }

        // Read real robot data from TestHandEye
        ArrayList<Mat> t_visuals = new ArrayList<>(); // Translation part of visual measurements
        System.out.println("Loaded " + (data.length / 4) + " measurement pairs from real robot data.");
        for (int i = 0; i < data.length / 4; i++) {
            // Visual measurement +0 and +1
            Mat t_vis = Mat.zeros(3, 1, CvType.CV_64F);
            t_vis.put(0, 0, data[i * 4 + 0]);
            t_visuals.add(t_vis);
        }

        Rotation3d rotation1 = HandEyeCalibration.estimateRollPitch(t_visuals, true);
        System.out.println(String.format("If right side up. Roll = %.3f, Pitch = %.3f, Yaw = %.3f\n  %s",
            rotation1.getX(),
            rotation1.getY(),
            rotation1.getZ(),
            rotation1.getQuaternion()));

        // Alternatively, if the camera is upside down, the camera normal would point "up"
        Rotation3d rotation2 = HandEyeCalibration.estimateRollPitch(t_visuals, false);
        System.out.println(String.format("If upside down. Roll = %.3f, Pitch = %.3f, Yaw = %.3f\n  %s",
            rotation2.getX(),
            rotation2.getY(),
            rotation2.getZ(),
            rotation2.getQuaternion()));

        // However, yaw here is fantom and must be adjusted further (to be relative to what?)
    }

}
