// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConfigReader;
import frc.robot.sensors.IMU;

/** Represents a swerve drive style drivetrain. */
public class SwerveChassis extends SubsystemBase {
    public final double kMaxSpeed;          // Theoretical speed limit
    public final double kMaxAngularSpeed;   // Same but angular
    public final double kMaxAcceleration;   // Tipping over or skidding whichever is less
    public final double kMaxAngularAcceleration;

    private final int nModules;
    private final SwerveModule[] swerveModules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator odometry;
    private final SwerveModulePosition[] currentPositions;
    private final IMU gyro;
    private final Field2d field = new Field2d();
    public boolean fieldRelative = false;

    // Maybe temporary, random dashboard data
    public double velocity = 0;


    /**
     * Swerve drivetrain, configurable by a JSON Config file that has to be present
     * in the 'deploy/config' directory
     * 
     * @param configName The config file name. The file must be a json file
     */
    public SwerveChassis(String configName) {
        var drivetrainConfig = new ConfigReader(configName);
        gyro = new IMU(drivetrainConfig.getAsSubReader("externalGyro"));
        kMaxSpeed = drivetrainConfig.getAsDouble("robotPhysics/maxSpeed");
        kMaxAngularSpeed = drivetrainConfig.getAsDouble("robotPhysics/maxTurnRate");
        kMaxAcceleration = drivetrainConfig.getAsDouble("robotPhysics/maxAcceleration");
        kMaxAngularAcceleration = drivetrainConfig.getAsDouble("robotPhysics/maxTurnAcceleration");

        var swerveConfigArray = drivetrainConfig.getAsJsonArray("swerveModules");
        nModules = swerveConfigArray.size();
        currentPositions = new SwerveModulePosition[nModules];
        swerveModules = new SwerveModule[nModules];
        var swerveLocations = new Translation2d[nModules];

        for (int i = 0; i < nModules; i++) {
            var swerveConfigReader = new ConfigReader(swerveConfigArray.get(i).getAsJsonObject());
            var oneModule = new SwerveModule(swerveConfigReader);
            swerveModules[i] = oneModule;
            swerveLocations[i] = oneModule.mountPoint;
            currentPositions[i] = oneModule.getPosition();
            addChild(oneModule.getName(), oneModule);
            SmartDashboard.putData(oneModule);
        }
        kinematics = new SwerveDriveKinematics(swerveLocations);

        // Odometry wrapper class that has functionality for cameras that report position with latency
        // Here we initialize it with current readings from the sensors and the origin pose
        // Reset it when the real pose becomes known, e.g. from vision or autonomous dead reconing
        odometry = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), currentPositions, Pose2d.kZero);
        SmartDashboard.putData(this);
        SmartDashboard.putData("Field", field);
    }

    public SwerveDrivePoseEstimator getOdometry() {
        return odometry;
    }

    /**
     * Builds the sendable for Network Tables
     * 
     * @param builder sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> swerveModules[0].getSteerDegrees(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> swerveModules[0].getDriveVelocity(), null);

        builder.addDoubleProperty("Front Right Angle", () -> swerveModules[1].getSteerDegrees(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> swerveModules[1].getDriveVelocity(), null);

        builder.addDoubleProperty("Back Left Angle", () -> swerveModules[2].getSteerDegrees(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> swerveModules[2].getDriveVelocity(), null);

        builder.addDoubleProperty("Back Right Angle", () -> swerveModules[3].getSteerDegrees(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> swerveModules[3].getDriveVelocity(), null);

        builder.addDoubleProperty("Robot Angle", this::getYaw, null);
        builder.addDoubleProperty("Velocity", () -> velocity, null);
    }

    /**
     * Get the current yaw of the robot
     * 
     * @return The yaw of the robot in degrees
     */
    public double getYaw() {
        return odometry.getEstimatedPosition().getRotation().getDegrees();
    }

    /**
     * Method to drive the robot using Linear Velocities
     *
     * @param vX  Speed of the robot in the X directions
     * @param vY  Speed of the robot in the Y directions
     * @param vW  The angular rate of spin of the robot
     * @param dT  The time period between calls to Periodic() functions
     */
    public void driveVelocities(ChassisSpeeds speeds, Time dT) {
        // Every drive cycle update odometry and get the best field pose estimate
        final var pose = odometry.getEstimatedPosition();

        velocity = speeds.vxMetersPerSecond + speeds.vyMetersPerSecond;

        if (fieldRelative) {
            // Robot always drives robot oriented. If we want to drive field relative we have to convert
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pose.getRotation());
        }
        var discretSpeeds = ChassisSpeeds.discretize(speeds, dT.in(Seconds));
        var swerveModuleStates = kinematics.toSwerveModuleStates(discretSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

        for (int i = 0; i < nModules; i++) {
            swerveModules[i].setDesiredState(swerveModuleStates[i]);
        }
    }

    /**
     * Method to drive the robot using percents of max speeds
     *
     * @param speedPercent  Speed of the robot in the x, y directions
     * @param spinPercent   The angular rate of spin of the robot
     * @param periodSeconds The time period between calls to Periodic() functions.
     */
    public void driveSpeedPercents(Translation2d speedPercent, double spinPercent, double periodSeconds) {
        var scaledSpeeds = new ChassisSpeeds(
            speedPercent.getX() * kMaxSpeed,
            speedPercent.getY() * kMaxSpeed,
            spinPercent * kMaxAngularSpeed);

        driveVelocities(scaledSpeeds, Seconds.of(periodSeconds));
    }

    /**
     * Method to drive the robot with CTRE's SwerveRequest
     *
     * @param swerveRequest Speeds of the robot in the x, y directions and rotation
     */
    public void applyRequest(SwerveRequest.FieldCentric swerveRequest, Time dT) {
        var speeds = new ChassisSpeeds(
            swerveRequest.VelocityX,
            swerveRequest.VelocityY,
            swerveRequest.RotationalRate
        );
        driveVelocities(speeds, dT);
    }

    private void updatePositions() {
        for (int i = 0; i < nModules; i++) {
            currentPositions[i] = swerveModules[i].getPosition();
        }
    }

    /** Updates the field relative position of the robot. */
    public Pose2d updateOdometry() {
        updatePositions();
        var pose = odometry.update(gyro.getRotation2d(), currentPositions);
        field.setRobotPose(pose);
        return pose;
    }

    public void resetOdometry(Pose2d pose) {
        updatePositions();
        odometry.resetPosition(gyro.getRotation2d(), currentPositions, pose);
    }

    public void activateFieldOriented() {
        odometry.resetRotation(Rotation2d.kZero);
        fieldRelative = true;
    }

    public void deactivateFieldOriented() {
        fieldRelative = false;
    }
}
