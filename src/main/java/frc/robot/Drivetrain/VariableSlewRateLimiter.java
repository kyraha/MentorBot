package frc.robot.Drivetrain;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class VariableSlewRateLimiter implements Sendable {

    private double positiveRateLimit;
    private double negativeRateLimit;
    private double prevVal;
    private double prevTime;

    public VariableSlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        this.positiveRateLimit = positiveRateLimit;
        this.negativeRateLimit = negativeRateLimit;
        this.prevVal = initialValue;
        this.prevTime = MathSharedStore.getTimestamp();
    }

    public VariableSlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, (double)0.0F);
    }

    public double calculate(double input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - this.prevTime;
        double delta = input - this.prevVal;
        this.prevVal += MathUtil.clamp(delta, this.negativeRateLimit * elapsedTime, this.positiveRateLimit * elapsedTime);
        this.prevTime = currentTime;
        return this.prevVal;
    }

    public double calculate(Rotation2d input) {
        double currentTime = MathSharedStore.getTimestamp();
        double elapsedTime = currentTime - this.prevTime;
        this.prevVal += MathUtil.clamp(getDelta(input.getRadians()), this.negativeRateLimit * elapsedTime, this.positiveRateLimit * elapsedTime);
        this.prevTime = currentTime;
        this.prevVal = MathUtil.angleModulus(this.prevVal);
        return this.prevVal;
    }

    public double lastValue() {
        return this.prevVal;
    }

    public void reset(double value) {
        this.prevVal = value;
        this.prevTime = MathSharedStore.getTimestamp();
    }

    public void updateValues(double positiveRateLimit, double negativeRateLimit) {
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
        return MathUtil.angleModulus(input - this.prevVal);
    }

    public Translation2d wrapAngle(Translation2d targetVector, CommandSwerveDrivetrain driveTrain) {
        Translation2d currentVector =
                new Translation2d(driveTrain.getKinematics().toChassisSpeeds(driveTrain.getState().ModuleStates).vxMetersPerSecond,
                        driveTrain.getKinematics().toChassisSpeeds(driveTrain.getState().ModuleStates).vxMetersPerSecond);
        Rotation2d delta = targetVector.getAngle().minus(currentVector.getAngle());
        return Math.abs(delta.getDegrees()) > 90.0 ?
                new Translation2d(-targetVector.getNorm(), targetVector.getAngle().rotateBy(Rotation2d.kPi))
                : new Translation2d(targetVector.getNorm(), targetVector.getAngle());
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Slew Rate Limiter");

        builder.addDoubleProperty("Positive Rate Limit", ()->positiveRateLimit, (x)->positiveRateLimit = x);
        builder.addDoubleProperty("Last Value", this::lastValue, null);
    }
}
