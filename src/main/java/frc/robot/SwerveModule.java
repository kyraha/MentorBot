// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.gson.JsonObject;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements Sendable {
  private static final double kMaxSteerAcceleration = 6;  // Rotations per sec per sec
  private static final double kMaxSteerVelocity = 3;      // Rotations per second

  private final TalonFX steerMotor; // the steering motor
  private final TalonFX driveMotor; // the driving motor
  private final CANcoder absEncoder; // the can encoder attached to the steering shaft

  private final double absEncoderOffsetRotations;
  private final String moduleName; // to tell modules apart, e.g. on the ShuffleBoard

  private final MotionMagicDutyCycle steerRequest;
  private final VelocityDutyCycle driveRequest;

  /**
   * Mount point of the swerve module in meters relative to an agreed "center" of the robot
   */
  public final Translation2d mountPoint;

  /**
   * Constructs a SwerveModule with a drive motor, steering motor, drive encoder and steering encoder.
   *
   * @param config A JSONObject type object of a certain structure that has all the configuration.
   * 
   * location - translation of the module from the robot's origin.
   * encoder/offset - the CANcoder reading in radians when it's at physical zero
   */
  public SwerveModule(JsonObject config) {
    moduleName = config.getAsJsonPrimitive("name").getAsString();
    var steerConfig = config.getAsJsonObject("steer");
    var driveConfig = config.getAsJsonObject("drive");
    var encoderConfig = config.getAsJsonObject("encoder");
    double wheelRadius = config.getAsJsonPrimitive("wheelRadius").getAsDouble(); // in meters
    absEncoderOffsetRotations = encoderConfig.getAsJsonPrimitive("offset").getAsDouble(); // in rotations
    var wrapConfig = new ClosedLoopGeneralConfigs();
    wrapConfig.ContinuousWrap = true;

    var steerSettings = new TalonFXConfiguration()
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(steerConfig.getAsJsonPrimitive("gearing").getAsDouble()))
      .withClosedLoopGeneral(wrapConfig)
      .withSlot0(new Slot0Configs()
        .withKP(steerConfig.getAsJsonPrimitive("kP").getAsDouble())
        .withKI(steerConfig.getAsJsonPrimitive("kI").getAsDouble())
        .withKD(steerConfig.getAsJsonPrimitive("kD").getAsDouble())
        .withKS(steerConfig.getAsJsonPrimitive("kS").getAsDouble())
        .withKV(steerConfig.getAsJsonPrimitive("kV").getAsDouble())
        .withKA(steerConfig.getAsJsonPrimitive("kA").getAsDouble())
      )
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicAcceleration(kMaxSteerAcceleration)
        .withMotionMagicCruiseVelocity(kMaxSteerVelocity)
      );

    var driveSettings = new TalonFXConfiguration()
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(driveConfig.getAsJsonPrimitive("gearing").getAsDouble() / (2.0 * Math.PI * wheelRadius)))
      .withSlot0(new Slot0Configs()
        .withKP(driveConfig.getAsJsonPrimitive("kP").getAsDouble())
        .withKI(driveConfig.getAsJsonPrimitive("kI").getAsDouble())
        .withKD(driveConfig.getAsJsonPrimitive("kD").getAsDouble())
        .withKS(driveConfig.getAsJsonPrimitive("kS").getAsDouble())
        .withKV(driveConfig.getAsJsonPrimitive("kV").getAsDouble())
        .withKA(driveConfig.getAsJsonPrimitive("kA").getAsDouble())
      );

    steerMotor = new TalonFX(steerConfig.getAsJsonPrimitive("can").getAsInt());
    driveMotor = new TalonFX(driveConfig.getAsJsonPrimitive("can").getAsInt());
    absEncoder = new CANcoder(encoderConfig.getAsJsonPrimitive("can").getAsInt());

    steerMotor.getConfigurator().apply(steerSettings);
    driveMotor.getConfigurator().apply(driveSettings);

    mountPoint = new Translation2d(
      config.getAsJsonObject("location").getAsJsonPrimitive("x").getAsDouble(),
      config.getAsJsonObject("location").getAsJsonPrimitive("y").getAsDouble());

    // Set the current reading of the steering angle from the abs encoder once and forever
    steerMotor.setPosition(absEncoder.getAbsolutePosition().getValueAsDouble() - absEncoderOffsetRotations);

    steerRequest = new MotionMagicDutyCycle(0);
    driveRequest = new VelocityDutyCycle(0);
  }

  /**
   * Builds the sendable for shuffleboard
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Swerve Module");
    builder.addStringProperty("Name", this::getName, null);
    builder.addDoubleProperty("X-offset", mountPoint::getX, null);
    builder.addDoubleProperty("Y-offset", mountPoint::getY, null);
    builder.addDoubleProperty("Mileage", this::getDrivePosition, null);
    builder.addDoubleProperty("Speed", this::getDriveVelocity, null);
    builder.addDoubleProperty("Azimuth", this::getSteerDegrees, null);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getSteerRotation2d());
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getSteerRotation2d());
  }

  public Rotation2d getSteerRotation2d() {
    return Rotation2d.fromRotations(steerMotor.getPosition().getValueAsDouble());
  }

  public double getSteerDegrees() {
    return getSteerRotation2d().getDegrees();
  }

  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble();
  }

  public double getDriveVelocity() {
    return driveMotor.getVelocity().getValueAsDouble();
  }

  public String getName() {
    return moduleName;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    if (state.speedMetersPerSecond == 0) {
      // If the new state is with speed=0 then simply stop and don't change steering
      driveMotor.setControl(driveRequest.withVelocity(0));
      return;
    }

    var currentAngle = getSteerRotation2d();

    // Optimize the reference state to avoid spinning further than 90 degrees
    state.optimize(currentAngle);
    steerMotor.setControl(steerRequest.withPosition(state.angle.getRotations()));

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(currentAngle).getCos();
    driveMotor.setControl(driveRequest.withVelocity(state.speedMetersPerSecond));
  }
}
