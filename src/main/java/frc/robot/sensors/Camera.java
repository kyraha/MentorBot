package frc.robot.sensors;

import java.nio.file.FileSystems;
import java.nio.file.Path;

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
    public AprilTagFieldLayout aprilTagFieldLayout;
    private PhotonCamera cam;
    private Transform3d robotToCam;
    PhotonPoseEstimator photonPoseEstimator;

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
        robotToCam = new Transform3d(new Translation3d(0.3, 0.1, 0.2), new Rotation3d(0,0.26,0.087)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

        // Construct PhotonPoseEstimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);

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
