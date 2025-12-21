package frc.robot;

import java.util.ArrayList;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.calib3d.Calib3d;

import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.sensors.HandEyeCalibration;

import static frc.robot.TestHandEye.data;
import static frc.robot.sensors.HandEyeCalibration.Rotation3dFromMat;

public class TestLinearRegression {
    static ArrayList<Mat> t_visuals = new ArrayList<>(); // Translation part of visual measurements
    static ArrayList<Mat> t_odos = new ArrayList<>();    // Translation part of odometry measurements
    static ArrayList<Mat> r_odos = new ArrayList<>();    // Rotation part of odometry measurements

    @BeforeAll
    static void setup() {
        // Ensure OpenCV is loaded
        OpenCvLoader.forceStaticLoad();

        // Read real robot data from TestHandEye
        for (int i = 0; i < data.length / 4; i++) {
            // Visual measurement +0 and +1
            Mat t_vis = Mat.zeros(3, 1, CvType.CV_64F);
            t_vis.put(0, 0, data[i * 4 + 0]);
            t_visuals.add(t_vis);

            // Odometry measurement +2 and +3
            Mat t_odo = Mat.zeros(3, 1, CvType.CV_64F);
            t_odo.put(0, 0, data[i * 4 + 2]);
            t_odos.add(t_odo);
            Mat r_odo = Mat.zeros(3, 3, CvType.CV_64F);
            r_odo.put(0, 0, data[i * 4 + 3]);
            r_odos.add(r_odo);
        }
    }

    /**
     * This test demonstrates how to find camera rotation using linear regression by making multiple
     * measurements of 3D points and then derive the rotation needed to align the camera's "down"
     * direction with the horizontal plane normal. Where the plane is what all the measured points belong to.
     * For this experiment, we assume that the target is located lower than the camera.
     */
    @Test
    void testRollPitch() {
        System.out.println("Loaded " + (data.length / 4) + " measurement pairs from real robot data.");

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

    @Test
    void testYaw() {
        System.out.println("Loaded " + (data.length / 4) + " measurement pairs from real robot data.");

        Rotation3d rp = HandEyeCalibration.estimateRollPitch(t_visuals, false);
        ArrayList<Point> mRotatedVisuals = new ArrayList<>();
        ArrayList<Point> mFlattenedOdos = new ArrayList<>();
        for (int i=0; i < t_visuals.size(); i++) {
            Mat t_vis = t_visuals.get(i);
            Mat r_odo = r_odos.get(i);
            Rotation3d r_odo_rot = Rotation3dFromMat(r_odo);
            // Rotate each visual measurement by the estimated roll/pitch + odometry rotation
            Translation2d tv = new Translation3d(
                -t_vis.get(0, 0)[0],
                -t_vis.get(1, 0)[0],
                -t_vis.get(2, 0)[0])
                .rotateBy(rp).rotateBy(r_odo_rot).toTranslation2d();

            mRotatedVisuals.add(new Point(tv.getX(), tv.getY()));
            Mat mOdo2d = t_odos.get(i);
            mFlattenedOdos.add(new Point(mOdo2d.get(0, 0)[0], mOdo2d.get(1, 0)[0]));
            // System.out.println(" " + tv.getX() + " " + tv.getY());
            // System.out.println(" " + t_odos.get(i).get(0, 0)[0] + " " + t_odos.get(i).get(1, 0)[0]);
        }
        MatOfPoint2f mpVis = new MatOfPoint2f();
        MatOfPoint2f mpOdo = new MatOfPoint2f();
        mpVis.fromList(mRotatedVisuals);
        mpOdo.fromList(mFlattenedOdos);
        Mat homography = Calib3d.findHomography(mpVis, mpOdo);
        System.out.println("Homography:\n" + homography.dump());

        // Extract yaw from homography
        double yaw = Math.atan2(homography.get(1, 0)[0], homography.get(0, 0)[0]);
        System.out.println(String.format("Estimated yaw = %.3f radians (%.2f degrees)", yaw, Math.toDegrees(yaw)));
        Rotation3d lasRotation3d = new Rotation3d(0, 0, yaw);
        Rotation3d finalRotation = rp.plus(lasRotation3d);
        System.out.println(String.format("Final Rotation: Roll = %.3f, Pitch = %.3f, Yaw = %.3f\n  %s",
            finalRotation.getX(),
            finalRotation.getY(),
            finalRotation.getZ(),
            finalRotation.getQuaternion()));
    }
}
