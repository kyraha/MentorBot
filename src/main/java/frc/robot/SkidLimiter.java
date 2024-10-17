package frc.robot;

// Copyright (c) FIRST and 3130 the ERRORS.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A class that limits the rate of change of a 2-dimensional input.
 * Useful for implementing sluggish joystick to prevent sudden moves,
 * especially for swerve drivetrains.
 * 
 * There are two different rate limits:
 * - Radial - is how fast the joystick can be moved radially, away or towards the  center
 * - Angular - is how fast (radians/second) the joystick can change the direction it is pointing to
 *      when it is at full extent. The effective angular limit is variable and calculated so it can
 *      be ignored when the joystick is close to its neutral position.
 * 
 * The input for calculate() is a Translation2d vector. It can be composed of x and y coordinates
 * of the operator input device. And the output of calculate() method is the filtered Translation2d
 * vector that can be further used to control a holonomic drive system.
 */
public class SkidLimiter {
    private double m_angularRateLimit;
    private double m_radialRateLimit;

    private Translation2d m_prevState;
    private double m_prevTime;

    /**
     * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
     * value.
     *
     * @param angularRateLimit The arch rate-of-change limit at max radius, in radians per
     *     second. This is expected to be strictly positive. Limit = 0 doesn't make sense.
     * @param radialRateLimit The radial rate-of-change limit, in units per
     *     second. This can be either positive number or 0 to disable radial limit.
     * @param initialValue The initial state. Also see reset()
     */
    public SkidLimiter(double angularRateLimit, double radialRateLimit, Translation2d initialValue) {
        m_angularRateLimit = angularRateLimit;
        m_radialRateLimit = radialRateLimit;
        m_prevState = initialValue;
        m_prevTime = MathSharedStore.getTimestamp();
    }

    /**
     * Creates a new SkidLimiter with the given angular rate limit and unlimited radial movement
     *
     * @param angularRateLimit The arch change per second limit.
     */
    public SkidLimiter(double angularRateLimit) {
        this(angularRateLimit, 0, new Translation2d());
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public Translation2d calculate(Translation2d input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - m_prevTime;
        m_prevState = calculateDry(input, elapsedTime);
        m_prevTime = currentTime;
        return m_prevState;

    }

    /**
     * Stateless (dry) calculations of the allowed new state using the given elapsed time.
     * Does not mutate the Limiter object, i.e. no changes to previous states
     *  
     * @param input The new desired two-dimentional vector (translation)
     * @param elapsedTime The time interval elapsed from the previous (current) state and new input
     * @return The new, allowed translation vector
     */
    public Translation2d calculateDry(Translation2d input, double elapsedTime) {
        double angularChangeLimit = m_angularRateLimit * elapsedTime;
        double radialChangeLimit = m_radialRateLimit * elapsedTime;

        double newNorm = input.getNorm();
        double prevNorm = m_prevState.getNorm();

        Rotation2d newAngle = input.getAngle();
        Rotation2d prevAngle = m_prevState.getAngle();

        if(prevNorm > angularChangeLimit / Math.PI) {
            // Our theory is that angular limit should be inversly proportional to the magnitude
            // The simplest formula would be "Const / Norm" for the starters <-- edit this statement if theory changes
            double allowedChangeRadians = angularChangeLimit / prevNorm;
            Rotation2d desiredRotation = newAngle.minus(prevAngle);
            double desiredAngleAbsRadians = Math.abs(desiredRotation.getRadians());
            if(desiredAngleAbsRadians > allowedChangeRadians) {
                // t is how far between the initial and end values we are. It should be bounded in [0, 1]
                double t = allowedChangeRadians / desiredAngleAbsRadians;
                newAngle = prevAngle.interpolate(newAngle, t);
            }
        }
        
        if(m_radialRateLimit > 0) {
            newNorm = prevNorm +
                MathUtil.clamp(newNorm - prevNorm, -radialChangeLimit, radialChangeLimit);
        }

        return new Translation2d(newNorm, newAngle);
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(Translation2d value) {
        m_prevState = value;
        m_prevTime = MathSharedStore.getTimestamp();
    }

    public double getAngularRateLimit() { return m_angularRateLimit; }
    public void setAngularRateLimit(double angularRateLimit) {
        this.m_angularRateLimit = angularRateLimit;
    }

    public double getRadialRateLimit() { return m_radialRateLimit; }
    public void setRadialRateLimit(double radialRateLimit) {
        this.m_radialRateLimit = radialRateLimit;
    }
}
