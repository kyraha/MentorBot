package frc.robot.SlewDriving;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;

public class ContinuousSlewRateLimiter {
    private double positiveRateLimit;
    private double negativeRateLimit;
    private double prevVal;
    private double prevTime;

    public ContinuousSlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
        this.prevVal = initialValue;
        this.prevTime = MathSharedStore.getTimestamp();
    }

    public ContinuousSlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, (double)0.0F);
    }

    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - this.prevTime;
        this.prevVal += MathUtil.clamp(
            getDelta(input),
            this.negativeRateLimit * elapsedTime,
            this.positiveRateLimit * elapsedTime);
        this.prevVal = MathUtil.angleModulus(this.prevVal);
        this.prevTime = currentTime;
        return this.prevVal;
    }

    public double lastValue() {
        return this.prevVal;
    }

    public void reset(double value) {
        this.prevVal = value;
        this.prevTime = MathSharedStore.getTimestamp();
    }

    public void updateLmitis(double positiveRateLimit, double negativeRateLimit) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
    }

    public double getElapsedTime() {
        return MathSharedStore.getTimestamp() - prevTime;
    }

    public double getDelta(double input) {
        double delta = input - this.prevVal;
        if(delta > Math.PI) {
            delta -= Math.PI * 2;
        } else if(delta < -Math.PI) {
            delta += Math.PI * 2;
        }
        return delta;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Slew Rate Limiter");

        builder.addDoubleProperty("Positive Rate Limit", () -> positiveRateLimit, null);
        builder.addDoubleProperty("Last Value", this::lastValue, null);
    }
}
