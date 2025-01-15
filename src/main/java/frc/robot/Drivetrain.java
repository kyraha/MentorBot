// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
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

  /**
   * Swerve drivetrain, configurable by a JSON Config file that has to be present in the 'deploy' directory
   * * DrivetrainConfigPrototypERR.json
   * * DrivetrainConfigShowstoperr.json
   * @param initialRotation2d The initial rotation reported by an external gyro, e.g. NavX
   */
  public Drivetrain(Rotation2d initialRotation2d) {
    var drivetrainConfig = ConfigReader.readConfig("DrivetrainConfigPrototypERR.json");
    var swerveModulesConfig = drivetrainConfig.getAsJsonObject().getAsJsonArray("swerveModules");
    nModules = swerveModulesConfig.size();
    currentPositions = new SwerveModulePosition[nModules];
    swerveModules = new SwerveModule[nModules];
    var swerveLocations = new Translation2d[nModules];

    for (int i=0; i < nModules; i++) {
      var configObject = swerveModulesConfig.get(i).getAsJsonObject();
      var oneModule = new SwerveModule(configObject);
      swerveModules[i] = oneModule;
      swerveLocations[i] = oneModule.mountPoint;
      currentPositions[i] = oneModule.getPosition();
      addChild(oneModule.getName(), oneModule);
    }
    kinematics = new SwerveDriveKinematics(swerveLocations);

    // odometry wrapper class that has functionality for cameras that report position with latency
    odometry = new SwerveDrivePoseEstimator(kinematics, initialRotation2d, currentPositions, new Pose2d());
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
  public Pose2d updateOdometry(Rotation2d externalRotation2d) {
    for (int i=0; i < nModules; i++) {
      currentPositions[i] = swerveModules[i].getPosition();
    }
    return odometry.update(externalRotation2d, currentPositions);
  }

}
