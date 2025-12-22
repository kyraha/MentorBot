package frc.robot.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * This class is a limiter of accelerations for driving a swerve robot.
 * It limits the acceleration of the robot based on the speed and angle of the turn.
 * Important: The "turn" here is the curvature in the robot's path, not the rotation of the robot.
 */
public class PelicanLimiter {
    private static final double kAccelFactor = 12.0 * 40.0 / 70.0; // Watts available per kg of robot
    private static final double kMaxLinearAccel = 8.0; // m/s^2
    private static final double kMaxLinearDecel = -10.0; // m/s^2
    private static final double kMaxSkidAccel = 9.8;

    private boolean isAngleReal = false;
    public final VariableSlewRateLimiter driveLimiter = new VariableSlewRateLimiter(kMaxLinearAccel, kMaxLinearDecel, 0);
    public final VariableSlewRateLimiter thetaLimiter = new VariableSlewRateLimiter(0).enableRotationalInput();

    /**
     * Check if the vector is virtually zero. This is purely mathematical deadband to avoid division by zero.
     * @param vector The 2D vector to check
     * @return True if the vector is within the deadband, false otherwise
     */
    private boolean isSmall(Translation2d vector) {
        return vector.getNorm() < 0.001;
    }

    /**
     * Calculate the limit for the linear acceleration based on the current speed.
     * The theory is that the robot can accelerate less when it's already moving fast.
     * @param speed The current speed
     * @return The top limit for the allowed linear acceleration
     */
    public static double getDiminishingLimit (double speed) {
        if (speed > kAccelFactor / kMaxLinearAccel) {
            return kAccelFactor / speed;
        }
        else {
            return kMaxLinearAccel;
        }
    }

    /**
     * Calculate the limit for the turn rate based on the current speed.
     * The robot can turn less sharply when it's moving fast to avoid skidding.
     * The theory is that the centripetal acceleration = V^2 / R
     * And R = V*dT / asin(dTheta) or = V*dT / dTheta for small angles
     * So a = V^2 / (V*dT/dTheta) = V*dTheta/dT cannot exceed the max skid acceleration
     * Therefore the rate limit for dTheta/dT = a_max / V
     * @param speed The current speed of the robot
     * @return The positive limit for the allowed turn rate
     */
    public static double getTurnLimit(double speed) {
        // To avoid singularity at zero speed, we define a minimum speed, a Plank speed
        // Robot can stop from this speed in Plank time, 0.02s, with max skid deceleration
        final double plankSpeed = kMaxSkidAccel * 0.02;
        if (speed > plankSpeed) {
            // if everything is positive, the speed is also strictly positive here
            return kMaxSkidAccel / speed;
        }
        else {
            // Otherwise it's a quantum realm. We don't go there and stop at Plank limit
            return kMaxSkidAccel / plankSpeed;
        }
    }

    /*
     * Augment the desired speeds with acceleration limits.
     * This method is the main function of the class, it calculates the new speeds based on the current speeds.
     * @param desiredSpeeds The desired speeds for the robot
     * @return The new speeds with acceleration limits
     */
    public ChassisSpeeds calculate(ChassisSpeeds desiredSpeeds) {
        Translation2d vector = new Translation2d(desiredSpeeds.vxMetersPerSecond, desiredSpeeds.vyMetersPerSecond);
        double rotation = desiredSpeeds.omegaRadiansPerSecond;
        if(isAngleReal) {
            // Robot was moving last cycle, we're in business
            if(!isSmall(vector)) {
                // Input is not zero. Important: there's no else statement after this if
                // Non-zero input has an angle so we can do math
                double cosine = Math.cos(thetaLimiter.getDelta(vector.getAngle().getRadians()));
                if(cosine > 0) {
                    // If turn is manageable (angle is "small"), keep driving
                    driveLimiter.updatePositiveLimit(getDiminishingLimit(driveLimiter.lastValue()));
                    // Throttle desired magnitude by cosine of the desired turn then calculate new magnitude
                    double mag = driveLimiter.calculate(vector.getNorm() * cosine);
                    // Define the limit for the angle based on the current speed
                    double turnLimit = getTurnLimit(mag);
                    thetaLimiter.updateLimits(turnLimit, -turnLimit);
                    //calculate limits for rotation (with -pi to pi bounds)
                    Rotation2d angle = thetaLimiter.calculate(vector.getAngle());
                    vector = new Translation2d(mag, angle);
                    return new ChassisSpeeds(vector.getX(), vector.getY(), rotation);
                }
            }

            // Joystick is neutral or reversed -> decelerate
            thetaLimiter.reset(thetaLimiter.lastValue());
            // Calculate new magnitude with unknown upper limit because we decelerate
            double newMag = driveLimiter.calculate(0);
            vector = new Translation2d(newMag, new Rotation2d(thetaLimiter.lastValue()));
            if(isSmall(vector)) {
                // New mag is finally zero means the robot stops, no angle anymore
                isAngleReal = false;
                vector = Translation2d.kZero;
            }
            return new ChassisSpeeds(vector.getX(), vector.getY(), rotation);
        }
        else {
            // Otherwise the robot wasn't moving last cycle
            if (isSmall(vector)) {
                // Input is virtually a zero vector, do nothing but keep track of time
                driveLimiter.reset(0);
                return desiredSpeeds;
            } else {
                // Robot starts moving, now the angle is real
                isAngleReal = true;
                Rotation2d angle = vector.getAngle();
                thetaLimiter.reset(angle.getRadians()); // Next cycle starts from this angle
                driveLimiter.updatePositiveLimit(kMaxLinearAccel);
                double mag = driveLimiter.calculate(vector.getNorm());
                vector = new Translation2d(mag, angle);
                return new ChassisSpeeds(vector.getX(), vector.getY(), rotation);
            }
        }
        // The code should never reach this point of no return
    }
}
