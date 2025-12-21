package frc.robot.sensors;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.CommandSwerveDrivetrain;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import java.util.ArrayList;
import java.util.List;


public  class HandEyeCalibration extends Command
{
    private final int onlyTargetID = 20; // The ID of the fiducial to use for calibration

    private final Camera camera;
    private final CommandSwerveDrivetrain drivetrain;
    private double stillTime;
    private double timeOfPrevMeasurement = 0;
    private Pose2d poseOfPrevMeasurement = new Pose2d();

    private ArrayList<Mat> tTcTranslations;
    private ArrayList<Mat> gTbTranslations;
    private Mat hTeTranslation;
    private ArrayList<Mat> tTcRotations;
    private ArrayList<Mat> gTbRotations;
    private Mat hTeRotation;

    public HandEyeCalibration(Camera camera, CommandSwerveDrivetrain drivetrain)
    {
        this.camera = camera;
        this.drivetrain = drivetrain;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        // addRequirements(this.camera);
    }

    @Override
    public void initialize()
    {
        tTcTranslations = new ArrayList<>();
        gTbTranslations = new ArrayList<>();
        tTcRotations = new ArrayList<>();
        gTbRotations = new ArrayList<>();

        hTeTranslation = new Mat(3, 1, CvType.CV_64F);
        hTeRotation = new Mat(3, 3, CvType.CV_64F);
        stillTime = MathSharedStore.getTimestamp();
        System.out.println("Starting to collect data for Hand-Eye Calibration. Time is " + stillTime);
    }

    @Override
    public void execute()
    {
        List<PhotonPipelineResult> results = camera.getResults();
        SwerveDriveState driveState = drivetrain.getState();

        if(results.isEmpty()) return;

        if(isMoving(driveState)) {
            // Robot is moving, so keep resetting the still time
            stillTime = MathSharedStore.getTimestamp();
            return;
        }

        // If all is good, take a measurement
        takeMeasurement(results.get(results.size() - 1), driveState);
    }
    
    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        if(!gTbRotations.isEmpty()) {
            Calib3d.calibrateHandEye(gTbRotations, gTbTranslations, tTcRotations, tTcTranslations, hTeRotation, hTeTranslation);
            System.out.println("Hand to Eye Translation Matrix =\n" + hTeTranslation.dump());
            System.out.println("Hand to Eye Rotation Matrix =\n" + hTeRotation.dump());
            double[] r_buffer = new double[9];
            hTeRotation.get(0, 0, r_buffer);
            Rotation3d camRot = new Rotation3d(new Matrix<N3,N3>(N3.instance, N3.instance, r_buffer));
            System.out.println("Camera Rotation: " + camRot.getX() + "," + camRot.getY() + "," + camRot.getZ() + ", angle: " + camRot.getAngle());
            System.out.println("Rot Matrix: " + camRot.toMatrix());
        }
    }

    public boolean isMoving(SwerveDriveState driveState) {
        return (
            Math.hypot(
                driveState.Speeds.vxMetersPerSecond,
                driveState.Speeds.vyMetersPerSecond) > 0.01 ||
            Math.abs(driveState.Speeds.omegaRadiansPerSecond) > 0.002
        );
    }

    public void takeMeasurement(PhotonPipelineResult visionResult, SwerveDriveState driveState) {
        double nowTime = MathSharedStore.getTimestamp();
        Pose2d nowPose = driveState.Pose;

        // is timing good?
        if(
            // Vision result is too stale, was before the robot became still
            visionResult.getTimestampSeconds() < stillTime + 1.0 ||
            // Or not enough time has passed since the last measurement
            nowTime - timeOfPrevMeasurement < 2.0 || (
            // Or the robot hasn't moved since the last measurement
                nowPose.getTranslation().getDistance(poseOfPrevMeasurement.getTranslation()) < 0.05 &&
                Math.abs(nowPose.getRotation().getRadians() - poseOfPrevMeasurement.getRotation().getRadians()) < Math.toRadians(5)
            )
        ) return;

        Mat camT = new Mat(3, 1, CvType.CV_64F);
        Mat camR = new Mat(3, 3, CvType.CV_64F);
        Mat odoT = new Mat(3, 1, CvType.CV_64F);
        Mat odoR = new Mat(3, 3, CvType.CV_64F);

        boolean notPresent = true;
        for (PhotonTrackedTarget target : visionResult.getTargets()) {
            if (target.getFiducialId() == onlyTargetID) {
                Transform3d camToTarget = target.getBestCameraToTarget();
                camT.put(0, 0, camToTarget.getTranslation().toVector().getData());
                camR.put(0, 0, camToTarget.getRotation().toMatrix().getData());
                notPresent = false;
                break;
            }
        }
        if (notPresent) return;

        // Otherwise, we are in business
        timeOfPrevMeasurement = nowTime;
        poseOfPrevMeasurement = nowPose;

        // The target is present, record all the measurements
        odoT.put(0, 0, new Translation3d(nowPose.getTranslation()).toVector().getData());
        odoR.put(0, 0, new Rotation3d(nowPose.getRotation()).toMatrix().getData());

        tTcTranslations.add(camT);
        tTcRotations.add(camR);
        gTbTranslations.add(odoT);
        gTbRotations.add(odoR);

        System.out.println(flatStringMat(camT, "Vis Trans, time=") + nowTime);
        System.out.println(flatStringMat(camR, "Vis Rotat"));
        System.out.println(flatStringMat(odoT, "Odo Trans"));
        System.out.println(flatStringMat(odoR, "Odo Rotat"));
    }

    static public String flatStringMat(Mat mat, String name) {
        StringBuilder sb = new StringBuilder();
        int size = mat.rows() * mat.cols();
        double[] d_buffer = new double[size];
        mat.get(0, 0, d_buffer);
        sb.append("{");
        int i = 0;
        for(double d : d_buffer) {
            sb.append((float)d);
            if(i < size - 1) sb.append(", ");
            i++;
        }
        sb.append("}, // ").append(name);
        return sb.toString();
    }

    /**
     * Find camera rotation (except yaw) using linear regression by making multiple
     * measurements of 3D points and then derive the rotation needed to align the camera's "down"
     * direction with the horizontal plane normal. Where the plane is what all the measured points belong to.
     * 
     * The plane equation is of the form: ax + by + cz + d = 0 (in camera coordinates)
     * For our purposes, we fix c = -1, so the equation becomes: ax + by - z + d = 0
     * Rearranging gives: z = d + ax + by
     * 
     * We can use ordinary least squares regression to find the coefficients a, b, and d.
     * 
     * Once we have the plane's normal vector (a, b, -1), we can compute the rotation
     * needed to align the camera's "down" vector (0, 0, -1) with the plane's normal.
     * 
     * @param tTcTranslations List of translation matrices from camera to target
     * @param targetIsBelowCamera True if the target is located below the camera from the camera POV
     * @return Rotation3d representing the roll and pitch of the camera (yaw is not determined here)
     */
    static public Rotation3d estimateRollPitch(ArrayList<Mat> tTcTranslations, boolean targetIsBelowCamera) {
        // We collected a bunch of sample data points (x, y, z)
        // Let's put them into 2 separate arrays for regression
        double[] z = new double[tTcTranslations.size()];
        double[][] xy = new double[tTcTranslations.size()][2];
        int index = 0;
        for(Mat tTc : tTcTranslations) {
            xy[index][0] = tTc.get(0, 0)[0];
            xy[index][1] = tTc.get(1, 0)[0];
            z[index] = tTc.get(2, 0)[0];
            index++;
        }

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

        // Camera's own "down" direction is simply (0, 0, -1), or (0, 0, 1) if target is above camera
        Vector<N3> cameraNormal = VecBuilder.fill(0, 0, targetIsBelowCamera ? -1 : 1);

        // Camera is rotated as if plane would be rotated to match the camera normal
        // i.e, the rotation direction is from true "down" (plane normal) to camera "down" (camera normal)
        return new Rotation3d(planeNormal, cameraNormal);

        // However, yaw here is fantom and must be adjusted further (to be relative to what?)
    }

    public static Rotation3d Rotation3dFromMat(Mat rMat) {
        double[] r_buffer = new double[9];
        rMat.get(0, 0, r_buffer);
        return new Rotation3d(new Matrix<N3,N3>(N3.instance, N3.instance, r_buffer));
    }
}
