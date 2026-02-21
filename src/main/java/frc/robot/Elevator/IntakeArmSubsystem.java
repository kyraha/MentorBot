// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Elevator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeArmSubsystem extends SubsystemBase {
  private static final int kIntakeLimitDIO = 1;
  private static final int kIntakeArmCAN = 42;
  private final TalonFX armMotor;

  private final DigitalInput limitSwitch;
  private boolean isZeroed = false;

  private final MotionMagicVoltage voltRequest;
  private final TalonFXConfiguration motorConfig;

  private double kG = 0.31;
  private double kV = 13.5;
  private double kA = 0.2;
  private double kP = 50;
  private double kI = 0;
  private double kD = 4;

  private static final double sensorToMechGearRatio = 100;
  private double offset = 0;

  private double targetAcceleration = 2;
  private double targetVelocity = 1;

  private double outPos = 0;
  private double inPos = 0.25;

  private SingleJointedArmSim simulatedArm;
  Timer simulationTimer;

  public IntakeArmSubsystem() {
    limitSwitch = new DigitalInput(kIntakeLimitDIO);

    armMotor = new TalonFX(kIntakeArmCAN);

    motorConfig = new TalonFXConfiguration();
    updateConfig();

    voltRequest = new MotionMagicVoltage(0);

    if (RobotBase.isSimulation()) {
      simulatedArm = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        sensorToMechGearRatio,
        5*0.25*0.25,          // Moment of inertia (Kg*m²)
        0.25,
        0,
        Math.PI/2,            // Max angle (radians)
        true,
        Math.PI/2,            // Starting position
        new double[]{Math.PI/500,Math.PI/900}           // Std diviations
      );
      simulationTimer = new Timer();
      simulationTimer.start();
    }
  }

  private void extendIntakeToSetpoint(double setpoint) {
    armMotor.setControl(voltRequest.withPosition(setpoint));
  }

  public void intakeOut() {
    extendIntakeToSetpoint(outPos);
  }

  public void intakeIn() {
    extendIntakeToSetpoint(inPos);
  }

  private double getkG() {return kG;}
  private double getkV() {return kV;}
  private double getkA() {return kA;}
  private double getkP() {return kP;}
  private double getkI() {return kI;}
  private double getkD() {return kD;}
  private void setkG(double value) {kG = value;}
  private void setkV(double value) {kV = value;}
  private void setkA(double value) {kA = value;}
  private void setkP(double value) {kP = value;}
  private void setkI(double value) {kI = value;}
  private void setkD(double value) {kD = value;}

  public void updateConfig() {
    motorConfig.Slot0
      .withKG(kG)
      .withKV(kV)
      .withKA(kA)
      .withKP(kP)
      .withKI(kI)
      .withKD(kD)
      .withGravityType(GravityTypeValue.Arm_Cosine)
      .withGravityArmPositionOffset(offset);
    motorConfig.MotorOutput
      .withNeutralMode(NeutralModeValue.Brake)
      .withInverted(InvertedValue.CounterClockwise_Positive);
    motorConfig.MotionMagic
      .withMotionMagicAcceleration(targetAcceleration)
      .withMotionMagicCruiseVelocity(targetVelocity);
    motorConfig.Feedback
      .withSensorToMechanismRatio(sensorToMechGearRatio);

    armMotor.getConfigurator().apply(motorConfig);
  }

  public double getPosition() {return armMotor.getPosition().getValueAsDouble();}
  public double getVelocity() {return armMotor.getVelocity().getValueAsDouble();}
  public double getAcceleration() {return armMotor.getAcceleration().getValueAsDouble();}

  public double getTargetAcceleration() {return targetAcceleration;}
  public void setTargetAcceleration(double value) {targetAcceleration = value;}

  public double getTargetVelocity() {return targetVelocity;}
  public void setTargetVelocity(double value) {targetVelocity = value;}

  public double getProfilePosition() {
    return armMotor.getClosedLoopReference().getValueAsDouble();
  }
  public double getProfileVelocity() {
    return armMotor.getClosedLoopReferenceSlope().getValueAsDouble();
  }

  public double getStatorCurrent() {return armMotor.getStatorCurrent().getValueAsDouble();}

  public double getOutPos() {return outPos;}
  public double getInPos() {return inPos;}
  public void setOutPos(double value) {outPos = value;}
  public void setInPos(double value) {inPos = value;}

  public boolean atLimit() {return limitSwitch.get();}

  public boolean isZeroed() {return isZeroed;}

  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Intake");

    builder.addBooleanProperty("is Zeroed", this::isZeroed, null);
    builder.addBooleanProperty("Limit Reached", this::atLimit, null);

    builder.addDoubleProperty("Arm Position (rots)", this::getPosition, null);
    builder.addDoubleProperty("Arm Velocity (rps)", this::getVelocity, null);
    builder.addDoubleProperty("Arm Acceleration (rpss)", this::getAcceleration, null);

    builder.addDoubleProperty("Target Acceleration (rpss)", this::getTargetAcceleration, this::setTargetAcceleration);
    builder.addDoubleProperty("Target Velocity (rps)", this::getTargetVelocity, this::setTargetVelocity);

    builder.addDoubleProperty("Profile Position (rots)", this::getProfilePosition, null);
    builder.addDoubleProperty("Profile Velocity (rps)", this::getProfileVelocity, null);

    builder.addDoubleProperty("Stator Current (A)", this::getStatorCurrent, null);

    builder.addDoubleProperty("kG", this::getkG, this::setkG);
    builder.addDoubleProperty("kV", this::getkV, this::setkV);
    builder.addDoubleProperty("kA", this::getkA, this::setkA);
    builder.addDoubleProperty("kP", this::getkP, this::setkP);
    builder.addDoubleProperty("kI", this::getkI, this::setkI);
    builder.addDoubleProperty("kD", this::getkD, this::setkD);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(atLimit() && !isZeroed()) {
      armMotor.setPosition(0);
      isZeroed = true;
    }
  }

  AngularVelocity simPrevVelocity = RadiansPerSecond.of(0);
  @Override
  public void simulationPeriodic() {
    Time dTime = Seconds.of(simulationTimer.get());
    simulationTimer.restart();

    // Only do anything if some time has elapsed from the last period
    if (dTime.gt(Seconds.of(0.001))) {
      TalonFXSimState motorSim = armMotor.getSimState();
      simulatedArm.setInputVoltage(motorSim.getMotorVoltage());
      simulatedArm.update(dTime.in(Seconds));

      Angle rawRotorPosition = Radians.of(simulatedArm.getAngleRads() * sensorToMechGearRatio);
      AngularVelocity rawRotorVelocity = RadiansPerSecond.of(simulatedArm.getVelocityRadPerSec() * sensorToMechGearRatio);
      AngularAcceleration rawRotorAcceleration = rawRotorVelocity.minus(simPrevVelocity).div(dTime);

      motorSim.setRawRotorPosition(rawRotorPosition);
      motorSim.setRotorVelocity(rawRotorVelocity);
      motorSim.setRotorAcceleration(rawRotorAcceleration);
    }
  }
}
