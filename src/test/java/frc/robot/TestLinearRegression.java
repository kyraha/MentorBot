package frc.robot;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

public class TestLinearRegression {
    @Test
    void test() {
        // The plane is defined by the equation ax + by + cz + d = 0,
        // where (a, b, c) is a normal vector
        // and d = -(ax0 + by0 + cz0) for some point (x0, y0, z0) on the plane.
        // Assume c = -1, then a regression equation is z = d + ax + by
        // and [d, a, b] are the regression parameters.

        // Pretend we collected a bunch of sample data points (x, y, z) with getBestCameraToTarget()
        double[] z = {
            0,
            1,
            2
        };
        double[][] xy = {
            {2, 0},
            {4, 1},
            {4, -1}
        };

        // Create a regression model and fit the data
        OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();
        regression.newSampleData(z, xy);
        double[] coefficients = regression.estimateRegressionParameters();
        System.out.println(
            String.format("Standard Error: % .4f; R^2: % .4f",
            regression.estimateRegressionStandardError(),
            regression.calculateRSquared()));

        double d = coefficients[0];
        double a = coefficients[1];
        double b = coefficients[2];
        double c = -1.0; // c is fixed to -1
        System.out.println(String.format("Plane equation: % .4fx + % .4fy + % .4fz + % .4f = 0", a, b, c, d));

        Vector<N3> cameraNormal = new Vector<N3>(Nat.N3());
        cameraNormal.set(0, 0, 0);
        cameraNormal.set(1, 0, 0);
        cameraNormal.set(2, 0, -1);
        Vector<N3> planeNormal = new Vector<N3>(Nat.N3());
        planeNormal.set(0, 0, a);
        planeNormal.set(1, 0, b);
        planeNormal.set(2, 0, c);
        Rotation3d rotation = new Rotation3d(planeNormal, cameraNormal);
        System.out.println(String.format("Roll: %f, Pitch: %f, Yaw: %f, Angle: %f", rotation.getX(), rotation.getY(), rotation.getZ(), rotation.getAngle()));
        System.out.println(rotation.getQuaternion().toString());
    }
}
