package frc.robot.sensors;

import java.nio.file.FileSystems;
import java.nio.file.Path;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;

public class Camera {
    private static final double[] cameraIntrinsics = { // Theoretical, needs calibration
        652.857105, 0.0, 406.000481, // fx, 0, cx
        0.0, 654.862118, 317.871379, // 0, fy, cy
        0.0, 0.0, 1.0
    };
    public static final Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F, Scalar.all(0));
//     distortion
// 0.155644
// -0.182449
// 0.001267
// 0.001276
// -0.367385
// -0.009738
// 0.008300
// 0.015187
    private static final double[] homographyValues = {
        -0.0001270542819333261,  0.0008182770377802222,  0.1862827299336119,
         0.0006939972013693427,  0.0001741107045706584, -0.5076447771203513,
         0.0006253018433804395, -0.0029524108712688280,  1.0};
    public static final Mat homographyMatrix = new Mat(3, 3, CvType.CV_64F, Scalar.all(0));

    //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static Transform3d robotToCamera = new Transform3d(
        new Translation3d(0.287, 0.275, 0.395),
        new Rotation3d(3.0042,0.2186,-0.2814+0.0268-0.0642));

    PhotonPoseEstimator photonPoseEstimator;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonCamera cam;

    static {
        // Initialize the homography matrix with predefined values
        homographyMatrix.put(0, 0, homographyValues);
        // Initialize the camera matrix with theoretical values
        cameraMatrix.put(0, 0, cameraIntrinsics);
    }

    public Camera(String customFieldName) {
        try {
            Path pathToLayout = FileSystems.getDefault().getPath(
                Filesystem.getDeployDirectory().toString(),
                "fields",
                customFieldName);

            // The field from AprilTagField JSON file
            aprilTagFieldLayout = new AprilTagFieldLayout(pathToLayout);
        }
        catch (Exception e) {
            throw new RuntimeException(String.format("Error reading filed file: %s", customFieldName), e);
        }

        //Forward Camera
        cam = new PhotonCamera("3130Camera");

        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCamera);

    }

    public void addVisionMeasurement(SwerveDrivePoseEstimator odometry) {
        for (var oneResult : cam.getAllUnreadResults()) {
            photonPoseEstimator.setReferencePose(odometry.getEstimatedPosition());
            var estiamtedOpt = photonPoseEstimator.update(oneResult);
            if (estiamtedOpt.isPresent()) {
                var pose3d = estiamtedOpt.get().estimatedPose;
                var pose2d = new Pose2d(pose3d.getX(), pose3d.getY(), pose3d.getRotation().toRotation2d());
                odometry.addVisionMeasurement(pose2d, estiamtedOpt.get().timestampSeconds);
            }
        }
    }

}
