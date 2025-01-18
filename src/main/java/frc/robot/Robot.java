// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {
  private final PS5Controller operController = new PS5Controller(0);
  private final Drivetrain chassis = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SkidLimiter skidLimiter = new SkidLimiter(2.5);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(13);

  @Override
  public void robotInit() {

  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Every drive cycle update odometry and get the best field pose estimate
    final var pose = chassis.updateOdometry();

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -MathUtil.applyDeadband(operController.getLeftY(), 0.04);

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -MathUtil.applyDeadband(operController.getLeftX(), 0.04);

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var wSpeed = -MathUtil.applyDeadband(operController.getRightX(), 0.02);

    final var move = skidLimiter.calculate(new Translation2d(xSpeed, ySpeed)).times(Drivetrain.kMaxSpeed);
    final var spin = rotLimiter.calculate(wSpeed) * Drivetrain.kMaxAngularSpeed;

    final ChassisSpeeds speeds = fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(move.getX(), move.getY(), spin, pose.getRotation())
      : new ChassisSpeeds(move.getX(), move.getY(), spin);
    chassis.drive(speeds, getPeriod());
  }
}
