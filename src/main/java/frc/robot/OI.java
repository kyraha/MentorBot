package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Elevator.ElevatorSubsystem;
import frc.robot.Flywheel.RunFlywheel;

public class OI {
    // Robot wide constants
    public static final double ROBOT_SPEED_LIMIT = 3.0; // m/s
    public static final double ROBOT_SPIN_LIMIT = 3.0; // rad/s

    public final CommandPS5Controller mainController;

    public OI(Robot robot) {
        mainController = new CommandPS5Controller(0);

        configureBindings(robot);
    }

    public ChassisSpeeds getSpeedsFromHID() {
        if (mainController.isConnected()) {
            // First and foremost, apply deadband to the joystick axes to remove noise
            double xAxis = MathUtil.applyDeadband(-mainController.getLeftY(), 0.04);
            double yAxis = MathUtil.applyDeadband(-mainController.getLeftX(), 0.04);
            double rotation = MathUtil.applyDeadband(-mainController.getRightX(), 0.04);

            // Second, square the inputs and then apply max speeds to get the real units
            xAxis *= Math.abs(xAxis) * ROBOT_SPEED_LIMIT;
            yAxis *= Math.abs(yAxis) * ROBOT_SPEED_LIMIT;
            rotation *= Math.abs(rotation) * ROBOT_SPIN_LIMIT;

            // Then construct the speeds object and return it
            return new ChassisSpeeds(xAxis, yAxis, rotation);
        }
        else {
            return new ChassisSpeeds();
        }
    }

    public void configureBindings(Robot robot) {
        mainController.R1().onTrue(robot.elevator.goToSetpointCommand(ElevatorSubsystem.Constants.kElevatorL4));
        mainController.square().onTrue(robot.elevator.goToSetpointCommand(ElevatorSubsystem.Constants.kElevatorL3));
        mainController.cross().onTrue(robot.elevator.goToSetpointCommand(ElevatorSubsystem.Constants.kElevatorL2));
        // mainController.triangle().onTrue(robot.elevator.goToSetpointCommand(ElevatorSubsystem.Constants.kElevatorL1));
        mainController.L1().onTrue(robot.elevator.goToSetpointCommand(ElevatorSubsystem.Constants.kElevatorMinHeight));
        mainController.circle().whileTrue(new RunFlywheel(robot.flywheel));
    }
}
