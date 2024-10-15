// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final AnalogGyro m_gyro;
  private final int nModules;
  private final SwerveModule[] swerveModules;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  public Drivetrain(AnalogGyro analogGyro) {
    m_gyro = analogGyro;
    JSONObject drivetrainConfig = ConfigReader.readConfig("DrivetrainConfig.json");
    JSONArray swerveModulesConfig = drivetrainConfig.getJSONArray("swerveModules");
    nModules = swerveModulesConfig.length();
    swerveModules = new SwerveModule[nModules];
    var swerveLocations = new Translation2d[nModules];

    for (int i=0; i < nModules; i++) {
      Object oneObject = swerveModulesConfig.get(i);
      swerveModules[i] = new SwerveModule((JSONObject) oneObject);
      swerveLocations[i] = swerveModules[i].mountPoint;
    }
    kinematics = new SwerveDriveKinematics(swerveLocations);

    odometry = new SwerveDriveOdometry(
      kinematics,
      m_gyro.getRotation2d(),
      getSwervePositions());

    m_gyro.reset();
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
  public void updateOdometry() {
    odometry.update(
        m_gyro.getRotation2d(),
        getSwervePositions());
  }

  public SwerveModulePosition[] getSwervePositions() {
    var returnArray = new SwerveModulePosition[nModules];
    for (int i=0; i < nModules; i++) {
      returnArray[i] = swerveModules[i].getPosition();
    }
    return returnArray;
  }
}
