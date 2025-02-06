package frc.robot.Drivetrain;

import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Watts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Energy;
import edu.wpi.first.units.measure.ImmutableMomentOfInertia;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Power;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.PS5Controller;

public class StickDriver {
    public static final Mass robotMass = Kilogram.of(65);
    public static final MomentOfInertia robotInertia = new ImmutableMomentOfInertia(robotMass.in(Kilogram)*0.5*0.5/2.0, 1, KilogramSquareMeters);

    private SwerveChassis chassis;
    private final LinearAcceleration maxAcceleration;
    private final AngularAcceleration maxSpinAcceleration;
    private final Power availablPower;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private SkidLimiter skidLimiter;
    private ChassisSpeeds ghostSpeeds;

    public StickDriver(SwerveChassis drivetrainToUse) {
        chassis = drivetrainToUse;
        maxAcceleration = MetersPerSecondPerSecond.of(chassis.kMaxAcceleration);
        maxSpinAcceleration = RadiansPerSecondPerSecond.of(chassis.kMaxAngularAcceleration);
        availablPower = Watts.of(3000);
        skidLimiter = new SkidLimiter(maxSpinAcceleration.magnitude(), maxAcceleration.magnitude(), Translation2d.kZero);
        initialize();
    }

    public void initialize() {
        // The following initialization would be done in Initialize() if it was a Command
        ghostSpeeds = new ChassisSpeeds(); // Assume starting with Sticks in neutral
    }

    public void drive(PS5Controller operController, double periodSeconds) {
        Time dT = Seconds.of(periodSeconds);

        // Get the x speed. We are inverting this because game controllers return
        // negative values when we push forward. Also forward is controller's Y
        final var xStick = -MathUtil.applyDeadband(operController.getLeftY(), 0.04);

        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Game controllers
        // return positive values when you pull to the right on X axis
        final var yStick = -MathUtil.applyDeadband(operController.getLeftX(), 0.04);

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Game controllers return positive values when you pull to
        // the right on their X axis
        final var wStick = -MathUtil.applyDeadband(operController.getRightX(), 0.04);

        LinearVelocity vx = MetersPerSecond.of(xStick * chassis.kMaxSpeed);
        LinearVelocity vy = MetersPerSecond.of(yStick * chassis.kMaxSpeed);
        AngularVelocity vw = RadiansPerSecond.of(wStick * chassis.kMaxAngularSpeed);

        var newSpeeds = new ChassisSpeeds(vx, vy, vw);
        newSpeeds = desaturateAccelerations(newSpeeds, dT);
        chassis.driveVelocities(newSpeeds, dT);
    }

    ChassisSpeeds desaturateAccelerations(ChassisSpeeds newSpeeds, Time dT) {
        ChassisSpeeds enegryLimited = desaturateEnergyConsumption(newSpeeds, dT);

        final var skidLimited = skidLimiter.calculate(
            new Translation2d(enegryLimited.vxMetersPerSecond, enegryLimited.vyMetersPerSecond),
            dT.in(Seconds));

        ghostSpeeds = new ChassisSpeeds(skidLimited.getX(), skidLimited.getY(), enegryLimited.omegaRadiansPerSecond);
        return ghostSpeeds;
    }

    /**
     * Limits accelerations by power limit
     * Formula is: V1 = sqrt(C1 + V0^2) and W1 = sqrt(C2 + W0^2)
     * where C1 = 2*Em/m and C2 = 2*Ei/I, so that Em + Ei = Emax
     * where Emax is available energy for this iteration (power * dT)
     *       m is the mass of the robot
     *       I is the moment of inertia of the robot in X/Y plane
     * Note: a solid disk's I = m * R^2 / 2
     * @param askedSpeeds ChassisSpeeds from a human input device
     * @param dT delta T, time interval of the periodic cycle
     * @return desaturated ChassisSpeeds that avoid brown-outs
     */
    ChassisSpeeds desaturateEnergyConsumption(ChassisSpeeds askedSpeeds, Time dT) {
        // P = m*(V1^2-V0^2)/(2*dT) + I*(W1^2-W0^2)/(2*dT)

        var newVelocityMetersPerSecond = new Translation2d(askedSpeeds.vxMetersPerSecond, askedSpeeds.vyMetersPerSecond);
        var oldVelocityMetersPerSecond = new Translation2d(ghostSpeeds.vxMetersPerSecond, ghostSpeeds.vyMetersPerSecond);
        var absNewVelocity = MetersPerSecond.of(newVelocityMetersPerSecond.getNorm());
        var absOldVelocity = MetersPerSecond.of(oldVelocityMetersPerSecond.getNorm());
        var linearEnergy = linearAccelerationEnergy(absNewVelocity, absOldVelocity);
        if (linearEnergy.magnitude() < 0) linearEnergy = Joules.of(0);

        var newAngularVelocity = RadiansPerSecond.of(askedSpeeds.omegaRadiansPerSecond);
        var oldAngularVelocity = RadiansPerSecond.of(ghostSpeeds.omegaRadiansPerSecond);
        var angularEnergy = angularAccelerationEnergy(newAngularVelocity, oldAngularVelocity);
        if (angularEnergy.magnitude() < 0) angularEnergy = Joules.of(0);

        var totalEnergyAsked = linearEnergy.plus(angularEnergy);
        var totalEnergyAvailable = availablPower.times(dT);
        var energyScaler = totalEnergyAvailable.div(totalEnergyAsked);

        if (energyScaler.magnitude() < 1.0) {
            // Only clamp if the scaler is below 1.0, do not stretch energy if not asked
            linearEnergy = linearEnergy.times(energyScaler);
            angularEnergy = angularEnergy.times(energyScaler);
            var scaledVelocityMetersPerSecond = newVelocityMetersPerSecond;
            var scaledVelocityRadiansPerSecond = newAngularVelocity;

            if (absNewVelocity.magnitude() > 0 && linearEnergy.magnitude() > 0) {
                var linearCTerm = linearEnergy.times(2).div(robotMass).magnitude();
                var linearVelocitySquared = absOldVelocity.magnitude() * absOldVelocity.magnitude();
                var linearScaler = MetersPerSecond.of(Math.sqrt(linearVelocitySquared + linearCTerm)).div(absNewVelocity);
                scaledVelocityMetersPerSecond = newVelocityMetersPerSecond.times(linearScaler.magnitude());
            }

            if (newAngularVelocity.magnitude() > 0 && angularEnergy.magnitude() > 0) {
                var angularCTerm = angularEnergy.times(2).div(robotInertia).magnitude();
                var angularVelocitySquared = oldAngularVelocity.magnitude() * oldAngularVelocity.magnitude();
                scaledVelocityRadiansPerSecond = RadiansPerSecond.of(Math.sqrt(angularVelocitySquared + angularCTerm));
            }

            return new ChassisSpeeds(
                scaledVelocityMetersPerSecond.getX(),
                scaledVelocityMetersPerSecond.getY(),
                scaledVelocityRadiansPerSecond.magnitude());
        }
        else {
            // Otherwise the energy is not saturated, we are good
            return askedSpeeds;
        }
    }

    public static Energy linearAccelerationEnergy(LinearVelocity vNew, LinearVelocity vOld) {
        // delta Energy = mass * acceleration * distance
        // acceleration = delta_V / delta_T
        // distance = avg_V * delta_T
        // Therefore, delta Energy = mass * delta_V * avg_V
        var deltaV = vNew.minus(vOld);
        var avgV = vNew.plus(vOld).div(2);
        var deltaE = robotMass.times(deltaV).times(avgV);
        return Joules.of(deltaE.magnitude());
    }

    public static Energy angularAccelerationEnergy(AngularVelocity wNew, AngularVelocity wOld) {
        // Same math as for linear acceleration energy but angular
        // Therefore, delta Energy = mass * delta_omega * avg_omega
        var deltaW = wNew.minus(wOld);
        var avgW = wNew.plus(wOld).div(2);
        var deltaE = robotInertia.times(deltaW).times(avgW);
        return Joules.of(deltaE.magnitude());
    }
}
