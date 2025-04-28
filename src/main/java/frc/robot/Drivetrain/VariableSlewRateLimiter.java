package frc.robot.Drivetrain;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class VariableSlewRateLimiter implements Sendable {

    private double positiveRateLimit;
    private double negativeRateLimit;
    private double prevVal;
    private double prevTime;
    private boolean m_continuous = false;
    private double m_maximumInput;
    private double m_minimumInput;

    public VariableSlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
        this.prevVal = initialValue;
        this.prevTime = MathSharedStore.getTimestamp();
    }

    public VariableSlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, (double)0.0F);
    }

    public VariableSlewRateLimiter enableContinuousInput(double minimumInput, double maximumInput) {
        this.m_continuous = true;
        this.m_minimumInput = minimumInput;
        this.m_maximumInput = maximumInput;
        return this;
    }

    public VariableSlewRateLimiter enableRotationalInput() {
        return enableContinuousInput(-Math.PI, Math.PI);
    }

    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - this.prevTime;
        double delta = getDelta(input);
        this.prevVal += MathUtil.clamp(delta, this.negativeRateLimit * elapsedTime, this.positiveRateLimit * elapsedTime);
        this.prevTime = currentTime;
        return this.prevVal;
    }

    public Rotation2d calculate(Rotation2d input) {
        double limited = calculate(input.getRadians());
        return Rotation2d.fromRadians(limited);
    }

    public double lastValue() {
        return this.prevVal;
    }

    public void reset(double value) {
        this.prevVal = value;
        this.prevTime = MathSharedStore.getTimestamp();
    }

    public void updateLimits(double positiveRateLimit, double negativeRateLimit) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
    }

    public void setPositiveRateLimit(double positiveRateLimit) {
        this.positiveRateLimit = positiveRateLimit;
    }

    public double getPositiveRateLimit() {
        return positiveRateLimit;
    }

    public double getElapsedTime() {
        return MathSharedStore.getTimestamp() - prevTime;
    }

    public double getDelta(double input) {
        double delta = input - this.prevVal;
        if (m_continuous) {
            double errorBound = (m_maximumInput - m_minimumInput) / 2.0;
            delta = MathUtil.inputModulus(delta, -errorBound, errorBound);
        }
        return delta;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Slew Rate Limiter");

        builder.addDoubleProperty("Positive Rate Limit", ()->positiveRateLimit, (x)->positiveRateLimit = x);
        builder.addDoubleProperty("Negative Rate Limit", ()->negativeRateLimit, (x)->negativeRateLimit = x);
        builder.addDoubleProperty("Last Value", this::lastValue, null);
    }
}
