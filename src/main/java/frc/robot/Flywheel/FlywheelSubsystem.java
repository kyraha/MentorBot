package frc.robot.Flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sim.PhysicsSim;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX leftShooter;
    private final TalonFX rightShooter;

    private final MotionMagicVelocityVoltage voltRequest;
    private final NeutralOut stopRequest = new NeutralOut();
    private final TalonFXConfiguration motorConfig;

    private double kS = 0.1;
    private double kV = 0.12;
    private double kA = 0.01;
    private double kP = 0.01;
    private double kI = 0;
    private double kD = 0;

    private double sensorToMechGearRatio = 1;

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

        voltRequest = new MotionMagicVelocityVoltage(0);
    }

    public void updatePID() {
        motorConfig.Slot0.kS = kS;
        motorConfig.Slot0.kV = kV;
        motorConfig.Slot0.kA = kA;
        motorConfig.Slot0.kP = kP;
        motorConfig.Slot0.kI = kI;
        motorConfig.Slot0.kD = kD;
        rightShooter.getConfigurator().apply(motorConfig);
    }

    public void revAtVelocity(double rotsPerSec) {
        rightShooter.setControl(voltRequest
            .withVelocity(rotsPerSec)
            .withAcceleration(40)
        );
    }

    public void stopShooter() {
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

    public double getkI() {
        return kI;
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

    public void setkI(double value) {
        kI = value;
    }

    public void setkD(double value) {
        kD = value;
    }

    public double getVelocity() {
        double rotsPerSec = rightShooter.getVelocity().getValueAsDouble();
        double radsPerSec = Units.rotationsToRadians(rotsPerSec);
        double metersPerSec = radsPerSec * Units.inchesToMeters(2);
        return metersPerSec;
    }

    public double getAcceleration() {
        double rotsPerSecSquared = rightShooter.getAcceleration().getValueAsDouble();
        double radsPerSecSquared = Units.rotationsToRadians(rotsPerSecSquared);
        double metersPerSecSquared = radsPerSecSquared * Units.inchesToMeters(2);
        return metersPerSecSquared;
    }

    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Shooter");

        builder.addDoubleProperty("Velocity mps", this::getVelocity, null);
        builder.addDoubleProperty("Acceleration mpss", this::getAcceleration, null);

        builder.addDoubleProperty("kV", this::getkV, this::setkV);
        builder.addDoubleProperty("kA", this::getkA, this::setkA);
        builder.addDoubleProperty("kP", this::getkP, this::setkP);
        builder.addDoubleProperty("kI", this::getkI, this::setkI);
        builder.addDoubleProperty("kD", this::getkD, this::setkD);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void simulationInit() {
        PhysicsSim.getInstance().addTalonFX(rightShooter, 0.0015);
    }

}
