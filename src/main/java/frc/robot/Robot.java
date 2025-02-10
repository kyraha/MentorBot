// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Drivetrain.FakeDriving;
import frc.robot.Drivetrain.StickDriver;
import frc.robot.Drivetrain.SwerveChassis;

public class Robot extends TimedRobot {
    // Available configurations:
    // DrivetrainConfigPrototypERR.json
    // DrivetrainConfigShowstoperr.json
    private static final String drivetrainConfigName = "DrivetrainConfigPrototypERR.json";

    public SwerveChassis chassis;
    public OI oi;
    private final StickDriver driver;
    private final Command autonomousCommand;

    public Robot() {
        chassis = new SwerveChassis(drivetrainConfigName);
        driver = new StickDriver(chassis);
        autonomousCommand = new FakeDriving(chassis);
    }

    /**
     * This driverStationConnected() is a softer initialization than robotInit()
     * Do stuff that needs to occur after the DS is connected, such as needing the alliance information.
     */
    @Override
    public void driverStationConnected() {
        // Whatever was in robotInit() before
        oi = new OI(this);
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
        driver.drive(oi.mainController, getPeriod());
    }

    /**
    * This function is called every 20 ms, no matter the mode.
    * This runs after the mode specific periodic functions, but before LiveWindow and
    * SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        chassis.updateOdometry();
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.
        CommandScheduler.getInstance().run();
    }

}
