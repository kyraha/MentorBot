package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.BeforeEach;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

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
        Translation2d ghost = testLimiter.calculateDry(new Translation2d(1, 0), 100);
        assertEquals(1, ghost.getX(), DELTA);
        assertEquals(0, ghost.getY(), DELTA);
    }

    @Test
    void testLimitArch() {
        double elapsedTime = 0.02; // Seconds
        double expectedRadians = ANGULAR_LIMIT * elapsedTime;
        Translation2d ghost = testLimiter.calculateDry(new Translation2d(1, Rotation2d.fromDegrees(90)), elapsedTime);
        // System.out.println("Arch Ghost radius: " + ghost.getNorm());
        // System.out.println("Arch Ghost radians: " + ghost.getAngle().getRadians());
        assertEquals(expectedRadians, ghost.getAngle().getRadians(), DELTA, "Wrong angle returned");
    }

    @Test
    void testLimitMagnitude() {
        double elapsedTime = 0.02; // Seconds
        double expectedNorm = 1.0 - RADIAL_LIMIT * elapsedTime;
        Translation2d ghost = testLimiter.calculateDry(new Translation2d(0, 0), elapsedTime);
        // System.out.println("Norm Ghost radius: " + ghost.getNorm());
        // System.out.println("Norm Ghost radians: " + ghost.getAngle().getRadians());
        assertEquals(expectedNorm, ghost.getNorm(), DELTA, "Wrong magnitude returned");
    }
}
