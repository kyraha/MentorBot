package frc.robot.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS5Controller;

public class DriveStick {
    private SwerveChassis chassis;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SkidLimiter skidLimiter = new SkidLimiter(2.5, 2, new Translation2d());
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(13);

    public DriveStick(SwerveChassis drivetrainToUse) {
        chassis = drivetrainToUse;
    }

    public void drive(PS5Controller operController, double periodSeconds) {
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
        final var wSpeed = -MathUtil.applyDeadband(operController.getRightX(), 0.04);

        final var move = skidLimiter.calculate(new Translation2d(xSpeed, ySpeed), periodSeconds);
        final var spin = rotLimiter.calculate(wSpeed) * SwerveChassis.kMaxAngularSpeed;

        chassis.driveSpeedPercents(move, spin, periodSeconds);
      }
}
