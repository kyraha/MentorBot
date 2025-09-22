package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Elevator.ElevatorSubsystem;

public class OI {
    // Robot wide constants
    public static final double ROBOT_SPEED_LIMIT = 3.0; // m/s
    public static final double ROBOT_SPIN_LIMIT = 3.0; // rad/s

    // Gamepad POV List
    public static final int POV_UNPRESSED = -1;
    public static final int POV_N = 0;
    public static final int POV_NE = 45;
    public static final int POV_E = 90;
    public static final int POV_SE = 135;
    public static final int POV_S = 180;
    public static final int POV_SW = 225;
    public static final int POV_W = 270;
    public static final int POV_NW = 315;

    public static class PS5 {
        public static final int BTN_SQUARE = 1;
        public static final int BTN_X = 2;
        public static final int BTN_CIRCLE = 3;
        public static final int BTN_TRIANGLE = 4;
        public static final int BTN_LBUMPER = 5;
        public static final int BTN_RBUMPER = 6;
    
        public static final int BTN_LJOYSTICK_PRESS = 11;
        public static final int BTN_RJOYSTICK_PRESS = 12;
    }

    public static class Xbox {
        public static final int BTN_A = 1;
        public static final int BTN_B = 2;
        public static final int BTN_X = 3;
        public static final int BTN_Y = 4;
    
        public static final int BTN_LBUMPER = 5;
        public static final int BTN_RBUMPER = 6;
        public static final int BTN_WINDOW = 7;
        public static final int BTN_MENU = 8;
        public static final int BTN_LJOYSTICK_PRESS = 9;
        public static final int BTN_RJOYSTICK_PRESS = 10;
    }

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
    }
}
