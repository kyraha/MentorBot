package frc.robot;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

public class TestLinearRegression {
    /**
     * This test demonstrates how find camera rotation using linear regression by making multiple
     * measurements of 3D points and then derive the rotation needed to align the camera's "down"
     * direction with the horizontal plane normal.
     * 
     * The plane equation is of the form: ax + by + cz + d = 0 (in camera coordinates)
     * For our purposes, we fix c = -1, so the equation becomes: ax + by - z + d = 0
     * Rearranging gives: z = d + ax + by
     * 
     * We can use ordinary least squares regression to find the coefficients a, b, and d.
     * 
     * Once we have the plane normal vector (a, b, -1), we can compute the rotation
     * needed to align the camera's "down" vector (0, 0, -1) with the plane normal.
     */
    @Test
    void testRollPitch() {
        // Pretend we collected a bunch of sample data points (x, y, z)
        // In reality, just drive around the same AprilTag and record a few getBestCameraToTarget()
        double[] z = {
            -0.0533,
            -0.343,
            -0.822
        };
        double[][] xy = {
            {0.487, -0.1498},
            {3.45, 1.39},
            {2.94, -1.295}
        };

        // Create a regression model and fit the data
        OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();
        regression.newSampleData(z, xy);
        System.out.println( // Just to see how good the fit is
            String.format("Standard Error: % .4f; R^2: % .4f",
            regression.estimateRegressionStandardError(),
            regression.calculateRSquared()));

        double[] coefficients = regression.estimateRegressionParameters();
        double d = coefficients[0];
        double a = coefficients[1];
        double b = coefficients[2];
        double c = -1.0; // c is fixed to -1 (why? read above)
        System.out.println(String.format("Plane equation: %.3fx %+.3fy %+.3fz %+.3f = 0", a, b, c, d));

        // The plane normal vector from the camera POV is (a, b, c)
        Vector<N3> planeNormal = VecBuilder.fill(a, b, c);

        // Camera's own "down" direction is simply (0, 0, -1)
        Vector<N3> cameraNormal = VecBuilder.fill(0, 0, -1);

        // Camera is rotated as if plane would be rotated to match the camera normal
        // i.e, the rotation direction is from true "down" (plane normal) to camera "down" (camera normal)
        Rotation3d rotation1 = new Rotation3d(planeNormal, cameraNormal);
        System.out.println(String.format("If right side up. Roll = %.3f, Pitch = %.3f, Yaw = %.3f\n  %s",
            rotation1.getX(),
            rotation1.getY(),
            rotation1.getZ(),
            rotation1.getQuaternion()));

        // Alternatively, if the camera is upside down, the camera normal would point "up"
        cameraNormal.set(2, 0, 1); // Set Z component to +1
        Rotation3d rotation2 = new Rotation3d(planeNormal, cameraNormal);
        System.out.println(String.format("If upside down. Roll = %.3f, Pitch = %.3f, Yaw = %.3f\n  %s",
            rotation2.getX(),
            rotation2.getY(),
            rotation2.getZ(),
            rotation2.getQuaternion()));

        // However, yaw here is fantom and must be adjusted further (to be relative to what?)
    }

}
