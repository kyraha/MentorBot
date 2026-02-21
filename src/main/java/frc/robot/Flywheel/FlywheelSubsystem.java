package frc.robot.Flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Power.PowerBroker;
import frc.robot.sim.PhysicsSim;

public class FlywheelSubsystem extends SubsystemBase {
    static private final double kMomentOfInertia = 1.0 * 0.05;  // mass * radius
    static private final double kMaxPower = 30 * 12;

    private final TalonFX leftShooter;
    private final TalonFX rightShooter;

    private final MotionMagicVelocityVoltage voltRequest;
    private final NeutralOut stopRequest = new NeutralOut();
    private final TalonFXConfiguration motorConfig;

    // Tune the k-values in this particular order
    private double kS = 0.1;
    private double kV = 0.12;
    private double kA = 0.2;
    private double kP = 0.5;
    private double kD = 0;

    private double sensorToMechGearRatio = 1;
    private double powerPriority;
    private PowerBroker powerBroker;
    private double acceleration;
    private boolean isOn;

    /** Creates a new Shooter. */
    public FlywheelSubsystem() {
        rightShooter = new TalonFX(32);

        leftShooter = new TalonFX(33);
        leftShooter.setControl(
            new Follower(rightShooter.getDeviceID(),
            MotorAlignmentValue.Opposed));

        motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfig.Feedback.SensorToMechanismRatio = sensorToMechGearRatio;
        motorConfig.MotionMagic.MotionMagicJerk = 120;

        updatePID();

        acceleration = 0;
        voltRequest = new MotionMagicVelocityVoltage(0);
        isOn = false;

        powerPriority = 1;
        powerBroker = new PowerBroker(() -> this.powerPriority, "flywheel");
    }

    public void updatePID() {
        motorConfig.Slot0.kS = kS;
        motorConfig.Slot0.kV = kV;
        motorConfig.Slot0.kA = kA;
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kD = kD;
        rightShooter.getConfigurator().apply(motorConfig);
    }

    private double getAllowedAcceleration(double power) {
        // Ideally the math should be in radians per second, but 2π cancels out as the return value is also in rotations/sec²
        double omega = rightShooter.getVelocity().getValueAsDouble();
        if (omega < 20) omega = 20;
        double momentum = omega * kMomentOfInertia;
        return power / momentum / 24;
    }

    public void revAtVelocity(double rotsPerSec) {
        isOn = true;
        double power = powerBroker.requestPower(kMaxPower);
        acceleration = getAllowedAcceleration(power);
        rightShooter.setControl(voltRequest
            .withVelocity(rotsPerSec)
            .withAcceleration(acceleration)
        );
    }

    public void stopShooter() {
        isOn = false;
        rightShooter.setControl(stopRequest);
    }

    public double getkV() {
        return kV;
    }

    public double getkA() {
        return kA;
    }

    public double getkP() {
        return kP;
    }

    public double getkD() {
        return kD;
    }

    public void setkV(double value) {
        kV = value;
    }

    public void setkA(double value) {
        kA = value;
    }

    public void setkP(double value) {
        kP = value;
    }

    public void setkD(double value) {
        kD = value;
    }

    public double getVelocity() {
        return rightShooter.getVelocity().getValueAsDouble();
    }

    public double getAcceleration() {
        return rightShooter.getAcceleration().getValueAsDouble();
    }

    public double getReferenceVelocity() {
        return rightShooter.getClosedLoopReference().getValueAsDouble();
    }

    public double getReferenceAcceleration() {
        return rightShooter.getClosedLoopReferenceSlope().getValueAsDouble();
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");

        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        builder.addDoubleProperty("Acceleration", this::getAcceleration, null);
        builder.addDoubleProperty("Ref Velocity", this::getReferenceVelocity, null);
        builder.addDoubleProperty("Ref Acceleration", this::getReferenceAcceleration, null);

        builder.addDoubleProperty("kV", this::getkV, this::setkV);
        builder.addDoubleProperty("kA", this::getkA, this::setkA);
        builder.addDoubleProperty("kP", this::getkP, this::setkP);
        builder.addDoubleProperty("kD", this::getkD, this::setkD);
    }

    @Override
    public void periodic() {
        if (isOn) {
            double power = powerBroker.requestPower(kMaxPower);
            double accel = getAllowedAcceleration(power);
            rightShooter.setControl(voltRequest.withAcceleration(accel));
        }
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(rightShooter, kMomentOfInertia/2);
        PhysicsSim.getInstance().addTalonFX(leftShooter, kMomentOfInertia/2);
    }

}
