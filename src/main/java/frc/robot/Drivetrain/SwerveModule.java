// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Drivetrain;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.ConfigReader;

public class SwerveModule implements Sendable {
  private static final double kMaxSteerAcceleration = 30;  // Rotations per sec per sec
  private static final double kMaxSteerVelocity = 18;      // Rotations per second

  private final TalonFX steerMotor; // the steering motor
  private final TalonFX driveMotor; // the driving motor
  private final CANcoder absEncoder; // the can encoder attached to the steering shaft

  private final TalonFXConfiguration steerSettings;
  private final TalonFXConfiguration driveSettings;

  private final double absEncoderOffsetRotations;
  private final String moduleName; // to tell modules apart, e.g. in the Network Tables

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
  public SwerveModule(ConfigReader swerveConfig) {
    moduleName = swerveConfig.getAsString("name");
    double wheelRadius = swerveConfig.getAsDouble("wheelRadius"); // in meters
    absEncoderOffsetRotations = swerveConfig.getAsDouble("encoder/offset"); // in rotations
    var wrapConfig = new ClosedLoopGeneralConfigs();
    wrapConfig.ContinuousWrap = true;

    steerSettings = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
      )
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(swerveConfig.getAsDouble("steer/gearing")))
      .withClosedLoopGeneral(wrapConfig)
      .withSlot0(new Slot0Configs()
        .withKP(swerveConfig.getAsDouble("steer/kP"))
        .withKI(swerveConfig.getAsDouble("steer/kI"))
        .withKD(swerveConfig.getAsDouble("steer/kD"))
        .withKS(swerveConfig.getAsDouble("steer/kS"))
        .withKV(swerveConfig.getAsDouble("steer/kV"))
        .withKA(swerveConfig.getAsDouble("steer/kA"))
      )
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicAcceleration(kMaxSteerAcceleration)
        .withMotionMagicCruiseVelocity(kMaxSteerVelocity)
      );

    driveSettings = new TalonFXConfiguration()
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(swerveConfig.getAsDouble("drive/gearing") / (2.0 * Math.PI * wheelRadius)))
      .withSlot0(new Slot0Configs()
        .withKP(swerveConfig.getAsDouble("drive/kP"))
        .withKI(swerveConfig.getAsDouble("drive/kI"))
        .withKD(swerveConfig.getAsDouble("drive/kD"))
        .withKS(swerveConfig.getAsDouble("drive/kS"))
        .withKV(swerveConfig.getAsDouble("drive/kV"))
        .withKA(swerveConfig.getAsDouble("drive/kA"))
      );

    steerMotor = new TalonFX(swerveConfig.getAsInt("steer/can/id"), swerveConfig.getAsString("steer/can/bus"));
    driveMotor = new TalonFX(swerveConfig.getAsInt("drive/can/id"), swerveConfig.getAsString("drive/can/bus"));
    absEncoder = new CANcoder(swerveConfig.getAsInt("encoder/can/id"), swerveConfig.getAsString("encoder/can/bus"));

    steerMotor.getConfigurator().apply(steerSettings);
    driveMotor.getConfigurator().apply(driveSettings);

    mountPoint = new Translation2d(
      swerveConfig.getAsDouble("location/x"),
      swerveConfig.getAsDouble("location/y"));

    // Set the current reading of the steering angle from the abs encoder once and forever
    steerMotor.setPosition(absEncoder.getAbsolutePosition().getValueAsDouble() - absEncoderOffsetRotations);

    steerRequest = new MotionMagicDutyCycle(0);
    driveRequest = new VelocityDutyCycle(0);
  }

  /**
   * Builds the sendable for Network Tables
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
    builder.addDoubleProperty("AbsDegrees", this::getAbsDegrees, null);
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

  public Rotation2d getAbsRotation2d() {
    return Rotation2d.fromRotations(absEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public double getAbsDegrees() {
    return getAbsRotation2d().getDegrees();
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

  public TalonFXConfiguration getSteerSettings() {
    return steerSettings;
  }

  public TalonFXConfiguration getDriveSettings() {
    return driveSettings;
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
