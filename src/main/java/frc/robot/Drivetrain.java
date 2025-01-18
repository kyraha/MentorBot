// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final int nModules;
  private final SwerveModule[] swerveModules;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDrivePoseEstimator odometry;
  private final SwerveModulePosition[] currentPositions;
  private final Pigeon2 gyro;

  /**
   * Swerve drivetrain, configurable by a JSON Config file that has to be present in the 'deploy' directory
   * * DrivetrainConfigPrototypERR.json
   * * DrivetrainConfigShowstoperr.json
   * @param initialRotation2d The initial rotation reported by an external gyro, e.g. NavX
   */
  public Drivetrain() {
    var drivetrainConfig = new ConfigReader("DrivetrainConfigPrototypERR.json");
    // The "type" must be "Pigeon2". Maybe extend this logic to use any devices. Maybe in the future
    var pigeonCAN = drivetrainConfig.getAsInt("externalGyro/can/id");
    var pigeonBus = drivetrainConfig.getAsString("externalGyro/can/bus");
    gyro = new Pigeon2(pigeonCAN, pigeonBus);

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
    builder.addDoubleProperty("Rotation", this::getAzimuth, null);
  }

  public double getAzimuth()
  {
    return odometry.getEstimatedPosition().getRotation().getDegrees();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param speeds Speed of the robot in the x, y directions and the angular rate (ChassisSpeeds).
   * @param periodSeconds The time period between calls to Periodic() functions.
   */
  public void drive(ChassisSpeeds speeds, double periodSeconds) {
    var discretSpeeds = ChassisSpeeds.discretize(speeds, periodSeconds);
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
