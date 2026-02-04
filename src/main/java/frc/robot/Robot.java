// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Drivetrain.CommandSwerveDrivetrain;
import frc.robot.Drivetrain.PelicanDriver;
import frc.robot.Drivetrain.TunerConstants;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.sensors.Camera;

public class Robot extends TimedRobot {
    public CommandSwerveDrivetrain chassis;
    public ElevatorSubsystem elevator;
    public OI oi;
    public Camera camera;
    final Telemetry logger = new Telemetry(OI.ROBOT_SPEED_LIMIT);

    // private final StickDriver driver;
    private final Command autonomousCommand;

    public Robot() {
        chassis = TunerConstants.createDrivetrain();
        elevator = new ElevatorSubsystem();
        autonomousCommand = Commands.none();
        camera = new Camera("2025-ERRshop-field.json");
        chassis.registerTelemetry(logger::telemeterize);
    }

    /**
     * This driverStationConnected() is a softer initialization than robotInit()
     * Do stuff that needs to occur after the DS is connected, such as needing the alliance information.
     */
    @Override
    public void driverStationConnected() {
        // Whatever was in robotInit() before
        oi = new OI(this);
        chassis.setDefaultCommand(new PelicanDriver(chassis, oi));
    }

    @Override
    public void autonomousInit() {
        // Initialize autonomous command(s)
        CommandScheduler.getInstance().schedule(autonomousCommand);
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
        // Periodically updates odometry with vision from the Camera
        //camera.addVisionMeasurement(chassis);

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.
        CommandScheduler.getInstance().run();
    }

}
