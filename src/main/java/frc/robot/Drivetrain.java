// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
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

  private final ArrayList<SwerveModule> swerveModules;
  private final SwerveDriveKinematics kinematics;

  private final SwerveDriveOdometry odometry;

  public Drivetrain(AnalogGyro analogGyro) {
    m_gyro = analogGyro;
    swerveModules = new ArrayList<>();
    JSONObject drivetrainConfig = ConfigReader.readConfig("DrivetrainConfig.json");
    JSONArray swerveModulesConfig = drivetrainConfig.getJSONArray("swerveModules");

    for (Object oneObject : swerveModulesConfig) {
      swerveModules.add(new SwerveModule((JSONObject) oneObject));
    }

    var swerveLocations = new Translation2d[swerveModules.size()];
    for (int i=0; i < swerveModules.size(); i++) {
      swerveLocations[i] = swerveModules.get(i).mountPoint;
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
    for (int i=0; i < swerveModules.size(); i++) {
      swerveModules.get(i).setDesiredState(swerveModuleStates[i]);
    }
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        m_gyro.getRotation2d(),
        getSwervePositions());
  }

  public SwerveModulePosition[] getSwervePositions() {
    var returnArray = new SwerveModulePosition[swerveModules.size()];
    for (int i=0; i < swerveModules.size(); i++) {
      returnArray[i] = swerveModules.get(i).getPosition();
    }
    return returnArray;
  }
}
