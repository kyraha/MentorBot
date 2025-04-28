package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.OI;

/**
 * This class is a limiter of accelerations for driving a swerve robot.
 * It limits the acceleration of the robot based on the speed and angle of the turn.
 * Important: The "turn" here is the curvature in the robot's path, not the rotation of the robot.
 */
public class PelicanLimiter {
    private static final double kAccelFactor = 12.0 * 40.0 / 70.0; // Watts available per kg of robot

    private boolean isAngleReal = false;
    private final double skidMaxAccel = 9.8;
    public final VariableSlewRateLimiter driveLimiter = new VariableSlewRateLimiter(OI.ROBOT_MAX_ACCEL, OI.ROBOT_MAX_DECEL, 0);
    public final VariableSlewRateLimiter thetaLimiter = new VariableSlewRateLimiter(0).enableRotationalInput();

    /**
     * Check if the vector is virtually zero. This is purely mathematical deadband to avoid division by zero.
     * @param vector The 2D vector to check
     * @return True if the vector is within the deadband, false otherwise
     */
    private boolean isWithinDeadband(Translation2d vector) {
        return vector.getNorm() < 0.001;
    }

    /**
     * Calculate the limit for the linear acceleration based on the current speed.
     * The theory is that the robot can accelerate less when it's already moving fast.
     * @param vector The 2D vector of the current speed
     * @return The top limit for the allowed linear acceleration
     */
    public static double getDiminishingLimit (double speed) {
        if (speed > kAccelFactor / OI.ROBOT_MAX_ACCEL) {
            return kAccelFactor / speed;
        }
        else {
            return OI.ROBOT_MAX_ACCEL;
        }
    }

    /*
     * Augment the desired speeds with acceleration limits.
     * This method is the main function of the class, it calculates the new speeds based on the current speeds.
     * @param desiredSpeeds The desired speeds for the robot
     * @return The new speeds with acceleration limits
     */
    public ChassisSpeeds accelLimitVectorDrive(ChassisSpeeds desiredSpeeds) {
        Translation2d vector = new Translation2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond);
        double rotation = desiredSpeeds.omegaRadiansPerSecond;
        if(isAngleReal) {
            // Robot was moving last cycle, we're in business
            if(!isWithinDeadband(vector)) {
                // Input is not zero. Important: there's no else statement after this
                // Non-zero input has an angle so we can do math
                double cosine = Math.cos(thetaLimiter.getDelta(vector.getAngle().getRadians()));
                if(cosine > 0) {
                    // If turn is manageable (angle is "small"), keep driving
                    driveLimiter.setPositiveRateLimit(getDiminishingLimit(driveLimiter.lastValue()));
                    // Throttle desired vector by angle turned before calculating new magnitude
                    double mag = driveLimiter.calculate(vector.getNorm() * cosine);
                    // Define the limit for the angle based on the current speed
                    double limit = skidMaxAccel / mag;
                    thetaLimiter.updateLimits(limit, -limit);
                    //calculate limits for rotation (with -pi to pi bounds)
                    Rotation2d angle = thetaLimiter.calculate(vector.getAngle());
                    vector = new Translation2d(mag, angle);
                    return new ChassisSpeeds(vector.getX(), vector.getY(), rotation);
                }
            }
            // Joystick is neutral or reversed - decelerate
            thetaLimiter.reset(thetaLimiter.lastValue());
            double newMag = driveLimiter.calculate(0);
            vector = new Translation2d(newMag, new Rotation2d(thetaLimiter.lastValue()));
            if(isWithinDeadband(vector)) {
                // New mag is finally zero means the robot stops, no angle anymore
                isAngleReal = false;
                vector = Translation2d.kZero;
            }
            return new ChassisSpeeds(vector.getX(), vector.getY(), rotation);
        }
        else {
            // Otherwise the robot was not moving last cycle
            if (isWithinDeadband(vector)) {
                // Input is virtually a zero vector, do nothing but keep track of time
                driveLimiter.reset(0);
                return desiredSpeeds;
            } else {
                // Robot starts moving, now the angle is real
                isAngleReal = true;
                thetaLimiter.reset(vector.getAngle().getRadians());
                driveLimiter.setPositiveRateLimit(OI.ROBOT_MAX_ACCEL);
                double mag = driveLimiter.calculate(vector.getNorm());
                vector = new Translation2d(mag, vector.getAngle());
                return new ChassisSpeeds(vector.getX(), vector.getY(), rotation);
            }
        }
        // The code should never reach this point of no return
    }
}
