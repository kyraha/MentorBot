// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ConfigReader;
import frc.robot.sensors.IMU;

/** Represents a swerve drive style drivetrain. */
public class SwerveChassis extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final int nModules;
  private final SwerveModule[] swerveModules;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator odometry;
  private final SwerveModulePosition[] currentPositions;
  private final IMU gyro;
  public boolean fieldRelative = false;

  /**
   * Swerve drivetrain, configurable by a JSON Config file that has to be present in the 'deploy' directory
   * * DrivetrainConfigPrototypERR.json
   * * DrivetrainConfigShowstoperr.json
   * @param initialRotation2d The initial rotation reported by an external gyro, e.g. NavX
   */
  public SwerveChassis() {
    var drivetrainConfig = new ConfigReader("DrivetrainConfigPrototypERR.json");
    gyro = new IMU(drivetrainConfig.getAsSubReader("externalGyro"));

    var swerveConfigArray = drivetrainConfig.getAsJsonArray("swerveModules");
    nModules = swerveConfigArray.size();
    currentPositions = new SwerveModulePosition[nModules];
    swerveModules = new SwerveModule[nModules];
    var swerveLocations = new Translation2d[nModules];

    for (int i=0; i < nModules; i++) {
      var swerveConfigReader = new ConfigReader(swerveConfigArray.get(i).getAsJsonObject());
      var oneModule = new SwerveModule(swerveConfigReader);
      swerveModules[i] = oneModule;
      swerveLocations[i] = oneModule.mountPoint;
      currentPositions[i] = oneModule.getPosition();
      addChild(oneModule.getName(), oneModule);
      SmartDashboard.putData(oneModule);
    }
    kinematics = new SwerveDriveKinematics(swerveLocations);

    // odometry wrapper class that has functionality for cameras that report position with latency
    odometry = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(), currentPositions, new Pose2d());
    SmartDashboard.putData(this);
  }

  /**
   * Builds the sendable for Network Tables
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Module");
    builder.addStringProperty("Name", this::getName, null);
    builder.addDoubleProperty("Yaw deg", this::getYaw, null);
  }

  public double getYaw()
  {
    return odometry.getEstimatedPosition().getRotation().getDegrees();
  }

  /**
   * Method to drive the robot using percents of max speeds
   *
   * @param speedPercent Speed of the robot in the x, y directions
   * @param spinPercent The angular rate of spin of the robot
   * @param periodSeconds The time period between calls to Periodic() functions.
   */
  public void driveSpeedPercents(Translation2d speedPercent, double spinPercent, double periodSeconds) {
    // Every drive cycle update odometry and get the best field pose estimate
    final var pose = updateOdometry();

    speedPercent = speedPercent.times(kMaxSpeed);
    spinPercent *= kMaxAngularSpeed;
    final ChassisSpeeds speeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(speedPercent.getX(), speedPercent.getY(), spinPercent, pose.getRotation())
      : new ChassisSpeeds(speedPercent.getX(), speedPercent.getY(), spinPercent);

    var discretSpeeds = ChassisSpeeds.discretize(speeds.times(kMaxSpeed), periodSeconds);
    var swerveModuleStates = kinematics.toSwerveModuleStates(discretSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    for (int i=0; i < nModules; i++) {
      swerveModules[i].setDesiredState(swerveModuleStates[i]);
    }
  }

  /** Updates the field relative position of the robot. */
  public Pose2d updateOdometry() {
    for (int i=0; i < nModules; i++) {
      currentPositions[i] = swerveModules[i].getPosition();
    }
    return odometry.update(gyro.getRotation2d(), currentPositions);
  }

}
