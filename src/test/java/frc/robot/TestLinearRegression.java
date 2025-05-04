package frc.robot;

import org.apache.commons.math3.stat.regression.OLSMultipleLinearRegression;
import org.junit.jupiter.api.Test;

public class TestLinearRegression {
    @Test
    void test() {
        // The plane is defined by the equation ax + by + cz + d = 0, where (a, b, c) is a normal vector
        // And d = -(ax0 + by0 + cz0) for some point (x0, y0, z0) on the plane.
        // Assume c = -1, then a regression equation is z = d + ax + by
        // and [d, a, b] are the regression parameters.
        OLSMultipleLinearRegression regression = new OLSMultipleLinearRegression();
        double[] z = {1, 1.1, 1.1, 1.3};
        double[][] xy = {
            {1, 0},
            {4, 1},
            {4, -1},
            {7, 3}
        };
        regression.newSampleData(z, xy);
        double[] coefficients = regression.estimateRegressionParameters();
        System.out.println("Standard Error: " + regression.estimateRegressionStandardError());

        System.out.println("Coefficients: ");
        for (double coefficient : coefficients) {
            System.out.println(coefficient);
        }

        double d = coefficients[0];
        double a = coefficients[1];
        double b = coefficients[2];
        double c = -1.0; // c is fixed to -1
        System.out.println("Normal vector: " + a + " " + b + " " + c);
    }
}
