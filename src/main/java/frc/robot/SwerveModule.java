// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.json.JSONObject;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveModule implements Sendable {
  private static final TrapezoidProfile.Constraints steerConstraints =
    new TrapezoidProfile.Constraints(13, 30);

  // TBD: Should we move these static constants to the config?
  private static final double kWheelRadius = 0.0508; // in meters
  private static final double steerGearRatio = 21.42857; // Checked 2/2/24 //150d/7d = 21.42857  checked 1/19
  private static final double driveGearRatio = 6.12; // Checked 2/2/24 //6.75  checked 1/19/23

  private static final double steerRotToRads = 2.0 * Math.PI / steerGearRatio;
  private static final double driveRotToMeters = 2.0 * Math.PI * kWheelRadius / driveGearRatio;

  private final TalonFX steerMotor; // the steering motor
  private final TalonFX driveMotor; // the driving motor
  private final CANcoder absEncoder; // the can encoder attached to the steering shaft
  private final double absEncoderOffsetRadians;
  private final String moduleName; // to tell modules apart, e.g. on the ShuffleBoard

  private final PIDController drivePIDController;
  private final ProfiledPIDController steerPIDController;
  private final SimpleMotorFeedforward drivingFeedforward;
  private final SimpleMotorFeedforward steeringFeedforward;

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
  public SwerveModule(JSONObject config) {
    moduleName = config.getString("name");
    absEncoderOffsetRadians = config.getJSONObject("encoder").getDouble("offset");
    mountPoint = new Translation2d(
        config.getJSONObject("location").getDouble("x"),
        config.getJSONObject("location").getDouble("y"));

    steerMotor = new TalonFX(config.getJSONObject("steer").getInt("can"));
    driveMotor = new TalonFX(config.getJSONObject("drive").getInt("can"));
    absEncoder = new CANcoder(config.getJSONObject("encoder").getInt("can"));

    // Setup the steering PID controller
    var steerPID = config.getJSONObject("steer");
    steerPIDController = new ProfiledPIDController(
      steerPID.getDouble("kP"),
      steerPID.getDouble("kI"),
      steerPID.getDouble("kD"), steerConstraints);
    steerPIDController.enableContinuousInput(-Math.PI, Math.PI); // wrap for circles
    steerPIDController.setTolerance(0.0025, 0.05); // at position tolerance
    steeringFeedforward = new SimpleMotorFeedforward(steerPID.getDouble("kS"), steerPID.getDouble("kV"));

    // Setup the drive PID controller
    var drivePID = config.getJSONObject("drive");
    drivePIDController = new PIDController(
      drivePID.getDouble("kP"),
      drivePID.getDouble("kI"),
      drivePID.getDouble("kD"));
      drivingFeedforward = new SimpleMotorFeedforward(drivePID.getDouble("kS"), drivePID.getDouble("kV"));

    resetSteeringPosition();
  }

  /**
   * Builds the sendable for shuffleboard
   * @param builder sendable builder
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(moduleName);
    builder.addDoubleProperty("X-offset", mountPoint::getX, null);
    builder.addDoubleProperty("Y-offset", mountPoint::getY, null);
    builder.addDoubleProperty("Mileage", this::getDrivePosition, null);
    builder.addDoubleProperty("Speed", this::getDriveVelocity, null);
    builder.addDoubleProperty("Azimuth", this::getSteerDegrees, null);
  }

  /**
   * Resets the steering motor position sensor to the current reading from the
   * absolute encoder. This operation is needed most likely only once at the power up.
   */
  public void resetSteeringPosition() {
    // Read absolute encoder position in rotations
    double absRotations = absEncoder.getAbsolutePosition().getValueAsDouble();
    // Subtract the offset (in rotations) to find the real steering angle
    double actualRotations = (absRotations - absEncoderOffsetRadians/(2.0*Math.PI));
    // And store the result into the steer motor scaled up by the gear ratio
    steerMotor.setPosition(actualRotations * steerGearRatio);
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
    return Rotation2d.fromRadians(steerMotor.getPosition().getValueAsDouble() * steerRotToRads);
  }

  public double getSteerDegrees() {
    return getSteerRotation2d().getDegrees();
  }

  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble() * driveRotToMeters;
  }

  public double getDriveVelocity() {
    return driveMotor.getVelocity().getValueAsDouble() * driveRotToMeters;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var currentAngle = getSteerRotation2d();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentAngle);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(currentAngle).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = drivingFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        steerPIDController.calculate(currentAngle.getRadians(), state.angle.getRadians());

    final double turnFeedforward =
        steeringFeedforward.calculate(steerPIDController.getSetpoint().velocity);

    driveMotor.setVoltage(driveOutput + driveFeedforward);
    steerMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
