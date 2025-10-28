package frc.robot.sensors;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain.CommandSwerveDrivetrain;
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
    private double stillTime = Double.MAX_VALUE;
    private double timeOfPrevMeasurement = 0;

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
        System.out.println("Starting to collect data for Hand-Eye Calibration");
    }

    @Override
    public void execute()
    {
        List<PhotonPipelineResult> results = camera.getResults();
        SwerveDriveState driveState = drivetrain.getState();

        if(isSlow(driveState)) {
            if(!results.isEmpty()) {
                takeMeasurement(results.get(results.size() - 1), driveState);
            }
        }
        else {
            // Robot is moving, so keep resetting the still time
            stillTime = MathSharedStore.getTimestamp();
        }
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
        }
    }

    public boolean isSlow(SwerveDriveState driveState) {
        return (
            driveState.Speeds.vxMetersPerSecond < 0.2 &&
            driveState.Speeds.vyMetersPerSecond < 0.2 &&
            Math.abs(driveState.Speeds.omegaRadiansPerSecond) < 0.02
        );
    }

    public void takeMeasurement(PhotonPipelineResult visionResult, SwerveDriveState driveState) {
        if(
            // Vision result's timestamp is after when the robot became still
            visionResult.getTimestampSeconds() > stillTime &&
            // And enough time has passed since the last measurement
            MathSharedStore.getTimestamp() - timeOfPrevMeasurement >= 0.5
        ) {
            Pose2d statePose = driveState.Pose;
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

            // The target is present, record all the measurements
            odoT.put(0, 0, new Translation3d(statePose.getTranslation()).toVector().getData());
            odoR.put(0, 0, new Rotation3d(statePose.getRotation()).toMatrix().getData());

            tTcTranslations.add(camT);
            tTcRotations.add(camR);
            System.out.println("Added Vision Vec " + camT.dump());
            System.out.println("Added Vision Mat " + camR.dump());

            gTbTranslations.add(odoT);
            gTbRotations.add(odoR);
            System.out.println("Added Odometry Vec " + odoT.dump());
            System.out.println("Added Odometry Mat " + odoR.dump());

            timeOfPrevMeasurement = MathSharedStore.getTimestamp();
        }
    }
}
