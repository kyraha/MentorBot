// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Robot extends TimedRobot {
  private final AnalogGyro analogGyro = new AnalogGyro(0);
  private final XboxController operController = new XboxController(0);
  private final Drivetrain chassis = new Drivetrain(analogGyro.getRotation2d());

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SkidLimiter skidLimiter = new SkidLimiter(2.5);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit() {
    ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
    tab.add(chassis);
  }

  @Override
  public void autonomousInit() {
    analogGyro.reset();
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    chassis.updateOdometry(analogGyro.getRotation2d());
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -MathUtil.applyDeadband(operController.getLeftY(), 0.02) * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -MathUtil.applyDeadband(operController.getLeftX(), 0.02) * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var wSpeed = -MathUtil.applyDeadband(operController.getRightX(), 0.02) * Drivetrain.kMaxAngularSpeed;

    final var move = skidLimiter.calculate(new Translation2d(xSpeed, ySpeed));
    final var spin = rotLimiter.calculate(wSpeed);

    final ChassisSpeeds speeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(move.getX(), move.getY(), spin, analogGyro.getRotation2d())
      : new ChassisSpeeds(move.getX(), move.getY(), spin);
    chassis.drive(speeds, getPeriod());
  }
}
