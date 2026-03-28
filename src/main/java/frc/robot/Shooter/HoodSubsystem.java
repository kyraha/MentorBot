// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Seconds;

import static java.util.Map.entry;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {

    public static class Constants {
        //Shooter Curves
        public static InterpolatingDoubleTreeMap distanceToAngle = InterpolatingDoubleTreeMap.ofEntries(
            entry(1.2, 0.0058333333),
            entry(1.5, 0.00702777778),
            entry(2.0, 0.02363888889),
            entry(2.5, 0.03336111111),
            entry(3.4, 0.034),
            entry(3.75, 0.0355),
            entry(4.15, 0.036)
        );

        // Maximum allowed stator current
        public static final double maxStatorCurrent = 40;
        public static final double sensorToMechGearRatio = 117.63;

        // Hood speed and acceleration in rotations per second or second squared
        public static final double magicVelocity = 20;
        public static final double magicAcceleration = 100;

        public static final int canMotorId = 18;
        public static final int dioLimitSwitchPort = 2;

        public static final int configRetries = 5;
    }

    private final TalonFX motor;
    private final DigitalInput bottomLimitSwitch;

    private final MotionMagicDutyCycle motionMagicRequest;
    private boolean isZeroed = false;
    private double currentSetpoint = 0;
    private TalonFXConfiguration talonConfig;
    private double duration = 0;

    public HoodSubsystem() {
        motor = new TalonFX(Constants.canMotorId);
        bottomLimitSwitch = new DigitalInput(Constants.dioLimitSwitchPort);
        motionMagicRequest = new MotionMagicDutyCycle(0);

        talonConfig = new TalonFXConfiguration();
        talonConfig.MotionMagic
            .withMotionMagicCruiseVelocity(Constants.magicVelocity)
            .withMotionMagicAcceleration(Constants.magicAcceleration);
        talonConfig.Slot0
            .withKG(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKP(100)
            .withKI(0)
            .withKD(0.0);
        talonConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake);
        talonConfig.Feedback
            .withSensorToMechanismRatio(Constants.sensorToMechGearRatio);

        for (int i=0; i < Constants.configRetries; i++) {
            StatusCode code = motor.getConfigurator().apply(talonConfig);
            if (code.isOK()) break;
            System.out.println("Hood Apply Config failed. Try "+i);
        }
    }

    /**
     * Emergency stop. Cancels any motion magic and sets the motor to 0.
     * Should only be used in emergencies.
     * During normal operation, it's OK to keep the Motion Magic running.
     */
    public void stop() {
        motor.set(0);
        currentSetpoint = 0;
        isZeroed = false;
    }

    public void goToSetpoint(double setpoint) {
        currentSetpoint = setpoint;
    }

    double m_lastSimTime;
    @Override
    public void simulationPeriodic() {
        final double currentTime = Utils.getCurrentTimeSeconds();
        double deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        updateSimState(deltaTime, RobotController.getBatteryVoltage());
    }

    private DCMotorSim simulator = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, Constants.sensorToMechGearRatio),
        DCMotor.getKrakenX60(1));
    private AngularVelocity previousSimVelocity;
    private void updateSimState(double deltaTime, double batteryVolts) {
        final TalonFXSimState simTalon = motor.getSimState();
        final DIOSim simBottomSwitch = new DIOSim(bottomLimitSwitch);
        simTalon.setSupplyVoltage(batteryVolts);
        simulator.setInputVoltage(simTalon.getMotorVoltage());
        simulator.update(deltaTime);

        double hoodPosition = simulator.getAngularPositionRad();
        AngularVelocity hoodVelocity = simulator.getAngularVelocity();
        AngularAcceleration hoodAcceleration = (hoodVelocity.minus(previousSimVelocity)).div(Time.ofBaseUnits(deltaTime, Seconds));

        simTalon.setRawRotorPosition(hoodPosition);
        simTalon.setRotorVelocity(hoodVelocity);
        simTalon.setRotorAcceleration(hoodAcceleration);

        simBottomSwitch.setValue(hoodPosition <= 0.03);

        previousSimVelocity = hoodVelocity;
    }

    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }

    public boolean brokeBottomLimitSwitch() {
        return !bottomLimitSwitch.get();
    }

    public double getMMTargetPosition() {
        return motor.getClosedLoopReference().getValueAsDouble();
    }

    public double getMMTargetVelocity() {
        return motor.getClosedLoopReferenceSlope().getValueAsDouble();
    }

    /**
     * Initializes the data we send on shuffleboard
     * Calls the default init sendable for Subsystem Bases
     * 
     * @param builder sendable builder
     */
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Hood");

        builder.addDoubleProperty("Position", this::getPosition, null);
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        builder.addBooleanProperty("Bottom Limit", this::brokeBottomLimitSwitch, null);
        builder.addBooleanProperty("Is Zeroed", ()->isZeroed, null);
        builder.addDoubleProperty("Setpoint", () -> this.currentSetpoint, null);
        builder.addDoubleProperty("Reference", this::getMMTargetPosition, null);
        builder.addDoubleProperty("RefVelocity", this::getMMTargetVelocity, null);
        builder.addDoubleProperty("Duration", () -> duration, null);
    }

    @Override
    public void periodic() {
        double ts = Timer.getFPGATimestamp();

        if (isZeroed) {
            // Normal operation. Do periodic stuff here if any

        }
        else {
            // Constantly monitor limit switches
            if (brokeBottomLimitSwitch()) {
                // Now we know where the zero is. Set to motion magic and stay there
                motor.setPosition(0);
                isZeroed = true;
                motor.setControl(motionMagicRequest.withPosition(currentSetpoint));
            }
            else {
                // Simply keep driving slowly down
                motor.set(-0.05);
            }
        }

        if (!Utils.isSimulation() && motor.getStatorCurrent().getValueAsDouble() > Constants.maxStatorCurrent) {
            // Emergency stop if the stator current exceeds the threshold
            stop();
        }

        duration = (Timer.getFPGATimestamp() - ts)*1000;
    }
}
