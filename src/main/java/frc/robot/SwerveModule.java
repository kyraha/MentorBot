// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveModule {
  private static final double kWheelRadius = 0.0508;
  public final static double steerGearRatio = 21.42857; // Checked 2/2/24 //150d/7d = 21.42857  checked 1/19
  public final static double driveGearRatio = 6.12; // Checked 2/2/24 //6.75  checked 1/19/23
  public static final double steerRotToRads = 2.0 * Math.PI / steerGearRatio;
  public final static double driveRotToMeters = 2.0 * kWheelRadius * Math.PI / driveGearRatio;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      2 * Math.PI; // radians per second squared

  private final TalonFX steerMotor; // the steering motor
  private final TalonFX driveMotor; // the driving motor
  private final CANcoder absEncoder; // the can encoder attached to the steering shaft
  private final double absEncoderOffset; 

  private final PIDController drivePIDController = new PIDController(1, 0, 0);

  private final ProfiledPIDController steerPIDController =
      new ProfiledPIDController(1.25, 0.05, 0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward drivingFeedforward = new SimpleMotorFeedforward(1, 2.0);
  private final SimpleMotorFeedforward steeringFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, steering motor, drive encoder and steering encoder.
   *
   * @param steerMotorCanId CAN ID for the steering motor.
   * @param driveMotorCanId CAN ID for the drive motor.
   * @param cancoderCanId CAN ID for the absolute steering encoder.
   * @param cancoderOffsetRotations the CANcoder reading when it's at physical zero
   */
  public SwerveModule(int steerMotorCanId, int driveMotorCanId, int cancoderCanId, double cancoderOffsetRotations) {
    steerMotor = new TalonFX(steerMotorCanId);
    driveMotor = new TalonFX(driveMotorCanId);
    absEncoder = new CANcoder(cancoderCanId);
    absEncoderOffset = cancoderOffsetRotations;

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    steerPIDController.enableContinuousInput(-Math.PI, Math.PI); // wrap for circles
    steerPIDController.setTolerance(0.0025, 0.05); // at position tolerance
    resetSteeringPosition();
  }

  void resetSteeringPosition() {
    double currentPosition = absEncoder.getAbsolutePosition().getValueAsDouble();
    steerMotor.setPosition(currentPosition - absEncoderOffset);
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
    //Rotation2d.fromRadians(m_absoluteEncoder.getPosition().getValueAsDouble()));
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
    var encoderRotation = getSteerRotation2d();

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond);

    final double driveFeedforward = drivingFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        steerPIDController.calculate(getSteerRotation2d().getRadians(), state.angle.getRadians());

    final double turnFeedforward =
        steeringFeedforward.calculate(steerPIDController.getSetpoint().velocity);

    driveMotor.setVoltage(driveOutput + driveFeedforward);
    steerMotor.setVoltage(turnOutput + turnFeedforward);
  }
}
