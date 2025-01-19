package frc.robot;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Drivetrain.SwerveChassis;

public class OI {
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

    public final PS5Controller mainController;

    public OI(SwerveChassis chassis) {
        mainController = new PS5Controller(0);

        new POVButton(mainController, POV_N).onTrue(
            new InstantCommand(() -> {
                chassis.activateFieldOriented();
            }, chassis)
        );

        new POVButton(mainController, POV_S).onTrue(
            new InstantCommand(() -> {
                chassis.deactivateFieldOriented();
            }, chassis)
        );
    }
}
