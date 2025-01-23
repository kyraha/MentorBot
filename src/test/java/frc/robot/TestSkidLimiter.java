package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Drivetrain.SkidLimiter;

/** Add your docs here. */
public class TestSkidLimiter {
    static final double DELTA = 0.001;
    static final double ANGULAR_LIMIT = 1.0; // Radians per second
    static final double RADIAL_LIMIT = 25; // Units per second
    SkidLimiter testLimiter;

    @BeforeEach // this method will run before each test
    void setup() {
        testLimiter = new SkidLimiter(
            ANGULAR_LIMIT,
            RADIAL_LIMIT,
            new Translation2d(1, Rotation2d.fromRadians(0)));
    }

    @Test
    void simpleTest() {
        Translation2d ghost = testLimiter.calculate(new Translation2d(1, 0), 100);
        assertEquals(1, ghost.getX(), DELTA);
        assertEquals(0, ghost.getY(), DELTA);
    }

    @Test
    void testLimitArch() {
        double elapsedTime = 0.02; // Seconds
        double expectedRadians = ANGULAR_LIMIT * elapsedTime;
        Translation2d ghost = testLimiter.calculate(new Translation2d(1, Rotation2d.fromDegrees(30)), elapsedTime);
        assertEquals(expectedRadians, ghost.getAngle().getRadians(), DELTA, "Wrong angle returned");
    }

    @Test
    void testLimitMagnitude() {
        double elapsedTime = 0.02; // Seconds
        double expectedNorm = RADIAL_LIMIT * elapsedTime;
        Translation2d ghost = testLimiter.calculate(new Translation2d(0, 0), elapsedTime);
        assertEquals(0, ghost.getNorm(), DELTA, "Wrong deceleration limit");

        testLimiter.reset(ghost);
        ghost = testLimiter.calculate(new Translation2d(0, 1), elapsedTime);
        assertEquals(expectedNorm, ghost.getNorm(), DELTA, "Wrong acceleration limit");
    }

    @Test
    void testContinious() {
        // 45 degrees is about 0.8 radians so let's iterate 8 times
        for (int i = 0; i < 8; i++) {
            double elapsedTime = 0.1; // Seconds
            Translation2d fullThrottle = new Translation2d(1,1);
            var ghost = testLimiter.calculate(fullThrottle, elapsedTime);
            assertEquals(1.0, ghost.getX(), DELTA, "X should stay at 1.0");
            assertEquals(1.4*i*elapsedTime, ghost.getY(), 0.15, "Y should gradually grow");
            testLimiter.reset(ghost);
        }
    }
}
