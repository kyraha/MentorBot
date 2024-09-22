package frc.robot;

// Copyright (c) FIRST and 3130 the ERRORS.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * A class that limits the rate of change of an input state. Useful for implementing sluggish joystick
 * to prevent sudden moves, especially for swerve srivetrains.
 */
public class SkidLimiter {
    private final double m_angularRateLimit;
    private final double m_radialRateLimit;
    private Translation2d m_prevState;
    private double m_prevTime;

    /**
     * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
     * value.
     *
     * @param angularRateLimit The arch rate-of-change limit at max radius, in radians per
     *     second. This is expected to be positive.
     * @param radialRateLimit The radial rate-of-change limit, in units per
     *     second. This is expected to be positive.
     * @param initialValue The initial value of the input.
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
     * @param rateLimit The arch change per second limit.
     */
    public SkidLimiter(double rateLimit) {
        this(rateLimit, 0, new Translation2d());
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
        Rotation2d newAngle = input.getAngle();
        if(m_prevState.getNorm() > m_angularRateLimit / Math.PI) {
            double angularLimit = m_angularRateLimit / m_prevState.getNorm();
            double desiredRotationRadians = Math.abs(input.getAngle().minus(m_prevState.getAngle()).getRadians());
            if(desiredRotationRadians > angularLimit) {
                double t = angularLimit / desiredRotationRadians;
                newAngle = m_prevState.getAngle().interpolate(input.getAngle(), t);
            }
        }
        
        double newRadius = input.getNorm();
        if(m_radialRateLimit > 0) {
            newRadius = m_prevState.getNorm() +
                MathUtil.clamp(
                    input.getNorm() - m_prevState.getNorm(),
                    0, m_radialRateLimit * elapsedTime);
        }
        return new Translation2d(newRadius, newAngle);
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
}
