// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drivetrain.FakeDriving;
import frc.robot.Drivetrain.SwerveChassis;
import frc.robot.SlewDriving.PelicanDrive;
import frc.robot.sensors.Camera;

public class Robot extends TimedRobot {
    // Available configurations:
    // DrivetrainConfigPrototypERR.json
    // DrivetrainConfigShowstoperr.json
    private static final String drivetrainConfigName = "DrivetrainConfigPrototypERR.json";

    public SwerveChassis chassis;
    public OI oi;
    public Camera camera;
    // private final StickDriver driver;
    private final Command autonomousCommand;
    private final Field2d field = new Field2d();

    public Robot() {
        chassis = new SwerveChassis(drivetrainConfigName, isReal());
        // driver = new StickDriver(chassis);
        autonomousCommand = new FakeDriving(chassis);
        camera = new Camera("2025-ERRshop-field.json");
        SmartDashboard.putData("Field", field);
        chassis.resetOdometry(new Pose2d(2,2, Rotation2d.kZero));
    }

    /**
     * This driverStationConnected() is a softer initialization than robotInit()
     * Do stuff that needs to occur after the DS is connected, such as needing the alliance information.
     */
    @Override
    public void driverStationConnected() {
        // Whatever was in robotInit() before
        oi = new OI(this);
        chassis.setDefaultCommand(new PelicanDrive(chassis, oi.mainController, getPeriod()));
    }

    @Override
    public void autonomousInit() {
        // Initialize autonomous command(s)
        autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
        
    }

    @Override
    public void teleopPeriodic() {
        // driver.drive(oi.mainController, getPeriod());
    }

    /**
    * This function is called every 20 ms, no matter the mode.
    * This runs after the mode specific periodic functions, but before LiveWindow and
    * SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        // Periodically updates the main pose estimator from the wheels position
        field.setRobotPose(chassis.updateOdometry());

        // Periodically updates odometry with vision from the Camera
        camera.addVisionMeasurement(chassis.getOdometry());

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.
        CommandScheduler.getInstance().run();
    }

}
