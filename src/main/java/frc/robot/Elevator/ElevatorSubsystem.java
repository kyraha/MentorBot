// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Elevator;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Power.PowerBroker;

public class ElevatorSubsystem extends SubsystemBase {
    public static class Constants {
        // Elevator heights in rotations of the Left Motor
        public static final double kElevatorMaxHeight = 45.8;
        public static final double kElevatorMinHeight = 8;
        public static final double kElevatorL1 = 14.8;
        public static final double kElevatorL2 = 18.8;
        public static final double kElevatorL3 = 29.1;
        public static final double kElevatorL4 = 45.3;
        public static final double kElevatorRangeMeters = 1.6;

        // 3-stage elevator quadruples the actual range hence the division by 4.0
        public static final double kRotationsPerMeter = kElevatorMaxHeight / (kElevatorRangeMeters / 4.0);
        public static final double kCarriegeMassKg = 15.45;

        // Maximum allowed stator current
        public static final double maxStatorCurrent = 40;

        // Elevator speed and acceleration in rotations per second or second squared
        private static double magicVelocity = 40;
        private static double magicAcceleration = 150;

        // Elevator PID values
        private static double slot0kG = 0.045;
        private static double slot0kP = Utils.isSimulation() ? 0.013 : 1;
        private static double slot0kI = 0;
        private static double slot0kD = Utils.isSimulation() ? 0.00005 : 0.025;

        public static final int canMotorLeft = 18;
        public static final int canMotorRight = 19;
        public static final int dioBottomLimitSwitch = 0;
        public static final int dioTopLimitSwitch = 2;
    }

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;
    private final DIOSim simBottomSwitch;
    private final DIOSim simTopSwitch;

    private final MotionMagicDutyCycle motionMagicRequest;
    private boolean isZeroed = false;
    private double powerPriority;
    private PowerBroker powerBroker;
    private double currentSetpoint = 0;
    private TalonFXConfiguration talonConfig;
    private double allowedVelocity;

    public ElevatorSubsystem() {
        leftMotor = new TalonFX(Constants.canMotorLeft);
        rightMotor = new TalonFX(Constants.canMotorRight);
        bottomLimitSwitch = new DigitalInput(Constants.dioBottomLimitSwitch);
        topLimitSwitch = new DigitalInput(Constants.dioTopLimitSwitch);
        motionMagicRequest = new MotionMagicDutyCycle(0);

        simBottomSwitch = new DIOSim(bottomLimitSwitch);
        simTopSwitch = new DIOSim(topLimitSwitch);

        allowedVelocity = Constants.magicVelocity;
        talonConfig = new TalonFXConfiguration();
        talonConfig.MotionMagic
            .withMotionMagicCruiseVelocity(allowedVelocity)
            .withMotionMagicAcceleration(Constants.magicAcceleration);
        // Current limits are not used for now. The current is monitored in periodic()
        // config.CurrentLimits
        //     .withStatorCurrentLimitEnable(true)
        //     .withStatorCurrentLimit(Constants.maxStatorCurrent);
        talonConfig.Slot0
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(Constants.slot0kG)
            .withKP(Constants.slot0kP)
            .withKI(Constants.slot0kI)
            .withKD(Constants.slot0kD);
        talonConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        leftMotor.getConfigurator().apply(talonConfig);
        rightMotor.setControl(new Follower(Constants.canMotorLeft, true));

        powerPriority = 1;
        powerBroker = new PowerBroker(() -> this.powerPriority);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Sets the elevator to a setpoint using motion magic
     * 
     * @param setpoint the setpoint to move to in rotations of the left motor
     * @return a command that will set the elevator to the setpoint
     */
    public Command goToSetpointCommand(double setpoint) {
        return runOnce(() -> goToSetpoint(setpoint));
    }

    /**
     * Emergency stop for the elevator. Cancels any motion magic and sets the motor to 0.
     * Should only be used in emergencies.
     * During normal operation, it's OK to keep the Motion Magic running.
     */
    public void stop() {
        leftMotor.set(0);
        currentSetpoint = 0;
        powerBroker.releasePower();
    }

    public void goToSetpoint(double setpoint) {
        if (isZeroed) {
            leftMotor.setControl(motionMagicRequest.withPosition(setpoint));
            currentSetpoint = setpoint;
        }
        else {
            // If not yet zeroed, we need to find the zero by slowly moving down instead
            leftMotor.set(-0.1);
            currentSetpoint = 0;
        }
    }

    private double metersToRotations(double meters) {
        return meters * Constants.kRotationsPerMeter;
    }

    private double rotationsToMeters(double rotations) {
        return rotations / Constants.kRotationsPerMeter;
    }

    private Notifier m_simNotifier;
    double m_lastSimTime;
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(0.005);
    }

    private ElevatorSim simElevator = new ElevatorSim(
        DCMotor.getFalcon500(2),
        9.1367,
        Constants.kCarriegeMassKg,
        0.0254,
        0,
        (Constants.kElevatorRangeMeters + 0.06) / 4.0,  // +6 cm for dead zones beyond the limit switches
        true,
        0.05);
    private double previousVelocity = 0;
    private void updateSimState(double deltaTime, double batteryVolts) {
        var simTalon = leftMotor.getSimState();
        simTalon.setSupplyVoltage(batteryVolts);
        simElevator.setInputVoltage(simTalon.getMotorVoltage());
        simElevator.update(deltaTime);

        double elevatorPosition = simElevator.getPositionMeters();
        double elevatorVelocity = simElevator.getVelocityMetersPerSecond();
        double elevatorAcceleration = (elevatorVelocity - previousVelocity) / deltaTime;

        simTalon.setRawRotorPosition(metersToRotations(elevatorPosition));
        simTalon.setRotorVelocity(metersToRotations(elevatorVelocity));
        simTalon.setRotorAcceleration(metersToRotations(elevatorAcceleration));

        simBottomSwitch.setValue(!simElevator.wouldHitLowerLimit(elevatorPosition - 0.03/4.0));
        simTopSwitch.setValue(simElevator.wouldHitUpperLimit(elevatorPosition + 0.03/4.0));

        previousVelocity = elevatorVelocity;
    }



    /**
     * Returns the maximum horizontal acceleration allowed by the elevator height
     * This is a function of the height of the 3-stage elevator
     * 
     * @return the maximum horizontal acceleration limit in m/s^2
     */
    public double getHorisontalAccelerationLimit() {
        var heightsMeters = 0.6 + getPosition() * Constants.kElevatorRangeMeters / Constants.kElevatorMaxHeight;
        double rg = 0.3 * 9.81;         // wheelbase/2 * g
        double C0 = 0.75 * 0.4;         // 3/4 of the mass of the robot is in the base
        double C1 = 0.25 * (1+2+3)/3;   // 1/4 of the mass is in the 3 stages of the elevator
        return rg / (C0 + C1 * heightsMeters);
    }

    public double getPosition() {
        return leftMotor.getPosition().getValueAsDouble();
    }

    public double getVelocity() {
        return leftMotor.getVelocity().getValueAsDouble();
    }

    public boolean brokeBottomLimitSwitch() {
        return !bottomLimitSwitch.get();
    }

    public boolean brokeTopLimitSwitch() {
        return topLimitSwitch.get();
    }

    /**
     * Initializes the data we send on shuffleboard
     * Calls the default init sendable for Subsystem Bases
     * 
     * @param builder sendable builder
     */
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Elevator");

        builder.addDoubleProperty("Position", this::getPosition, null);
        builder.addDoubleProperty("Velocity", this::getVelocity, null);
        builder.addBooleanProperty("Bottom Limit", this::brokeBottomLimitSwitch, null);
        builder.addBooleanProperty("Top Limit", this::brokeTopLimitSwitch, null);
        builder.addBooleanProperty("Is Zeroed", ()->isZeroed, null);
        builder.addDoubleProperty("Allowed Velo", () -> allowedVelocity, null);
        builder.addDoubleProperty("Setpoint", () -> this.currentSetpoint, null);
    }

    private boolean aboveBottomLimitSwitch = true;
    @Override
    public void periodic() {
        // Constantly monitor limit switches and the stator current
        if (brokeBottomLimitSwitch()) {
            if (aboveBottomLimitSwitch) {
                aboveBottomLimitSwitch = false;
                stop();
                leftMotor.setPosition(0);
                isZeroed = true;
            }
        }
        else {
            aboveBottomLimitSwitch = true;
        }

        if (brokeTopLimitSwitch()) {
            stop();
        }

        if (!Utils.isSimulation() && leftMotor.getStatorCurrent().getValueAsDouble() > Constants.maxStatorCurrent) {
            stop();
        }

        double power = rotationsToMeters(Constants.magicVelocity) * Constants.kCarriegeMassKg * 9.8;
        if (getPosition() < currentSetpoint - 1) {
            power = powerBroker.requestPower(power);
            allowedVelocity = metersToRotations(power / Constants.kCarriegeMassKg / 9.8);
        }
        else {
            powerBroker.releasePower();
            allowedVelocity = Constants.magicVelocity;
        }
        talonConfig.MotionMagic.withMotionMagicCruiseVelocity(allowedVelocity);
        leftMotor.getConfigurator().apply(talonConfig);
    }
}
