package frc.robot;

import static edu.wpi.first.units.Units.Joules;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Drivetrain.StickDriver;

public class TestDriver {

    @Test
    void testLinearAccelerationEnergy() {
        double v0mps = 1.0;
        double v1mps = 2.0;
        double e0joules = StickDriver.robotMass.in(Kilograms) * (v0mps*v0mps) / 2.0;
        double e1joules = StickDriver.robotMass.in(Kilograms) * (v1mps*v1mps) / 2.0;
        var expectedLAccE = e1joules - e0joules;

        LinearVelocity v0 = MetersPerSecond.of(v0mps);
        LinearVelocity v1 = MetersPerSecond.of(v1mps);
        var laccEnergy = StickDriver.linearAccelerationEnergy(v1, v0);
        assertEquals(expectedLAccE, laccEnergy.in(Joules), 0.1);
    }

    @Test
    void testAngularAccelerationEnergy() {
        double w0rps = 1.0;
        double w1rps = 2.0;
        double e0joules = StickDriver.robotInertia.in(KilogramSquareMeters) * (w0rps*w0rps) / 2.0;
        double e1joules = StickDriver.robotInertia.in(KilogramSquareMeters) * (w1rps*w1rps) / 2.0;
        var expectedLAccE = e1joules - e0joules;

        AngularVelocity w0 = RadiansPerSecond.of(w0rps);
        AngularVelocity w1 = RadiansPerSecond.of(w1rps);
        var aaccEnergy = StickDriver.angularAccelerationEnergy(w1, w0);
        assertEquals(expectedLAccE, aaccEnergy.in(Joules), 0.1);
    }

}
