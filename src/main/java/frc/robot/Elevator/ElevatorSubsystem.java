// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
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

        // Maximum allowed stator current
        public static final double maxStatorCurrent = 40;

        // Elevator speed and acceleration in rotations per second or second squared
        private static double magicVelocity = 100;
        private static double magicAcceleration = 120;

        // Elevator PID values
        private static double slot0kG = 0.045;
        private static double slot0kP = 1;
        private static double slot0kI = 0;
        private static double slot0kD = 0.025;

        public static final int canMotorLeft = 18;
        public static final int canMotorRight = 19;
        public static final int dioBottomLimitSwitch = 0;
        public static final int dioTopLimitSwitch = 2;
    }

    private final TalonFX leftMotor;
    private final TalonFX rightMotor;
    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;

    private final MotionMagicDutyCycle motionMagicRequest;
    private boolean isZeroed = false;
    private double powerPriority;
    private PowerBroker powerBroker;

    public ElevatorSubsystem() {
        leftMotor = new TalonFX(Constants.canMotorLeft);
        rightMotor = new TalonFX(Constants.canMotorRight);
        bottomLimitSwitch = new DigitalInput(Constants.dioBottomLimitSwitch);
        topLimitSwitch = new DigitalInput(Constants.dioTopLimitSwitch);
        motionMagicRequest = new MotionMagicDutyCycle(0);

        var config = new TalonFXConfiguration();
        config.MotionMagic
            .withMotionMagicCruiseVelocity(Constants.magicVelocity)
            .withMotionMagicAcceleration(Constants.magicAcceleration);
        // Current limits are not used for now. The current is monitored in periodic()
        // config.CurrentLimits
        //     .withStatorCurrentLimitEnable(true)
        //     .withStatorCurrentLimit(Constants.maxStatorCurrent);
        config.Slot0
            .withGravityType(GravityTypeValue.Elevator_Static)
            .withKG(Constants.slot0kG)
            .withKP(Constants.slot0kP)
            .withKI(Constants.slot0kI)
            .withKD(Constants.slot0kD);
        config.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        leftMotor.getConfigurator().apply(config);
        rightMotor.setControl(new Follower(Constants.canMotorLeft, true));

        powerPriority = 1;
        powerBroker = new PowerBroker(() -> this.powerPriority);
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
    }

    public void goToSetpoint(double setpoint) {
        if (isZeroed) {
            leftMotor.setControl(motionMagicRequest.withPosition(setpoint));
        }
        else {
            // If not yet zeroed, we need to find the zero by slowly moving down instead
            leftMotor.set(-0.1);
        }
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
    }

    @Override
    public void periodic() {
        // Constantly monitor limit switches and the stator current
        if (brokeBottomLimitSwitch()) {
            stop();
            // On the broken bottom limit switch set zero even if already zeroed
            if (getPosition() != 0) {
                leftMotor.setPosition(0);
                isZeroed = true;
            }
        }
        if (brokeTopLimitSwitch()) {
            stop();
        }
        if (leftMotor.getStatorCurrent().getValueAsDouble() > Constants.maxStatorCurrent) {
            stop();
        }
    }
}
