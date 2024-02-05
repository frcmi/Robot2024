package frc.lib.math;

import edu.wpi.first.math.geometry.Translation2d;

public class Transformations {
    /**
     * Retrieves the squared length of a 2D vector
     * @param v The vector
     * @return The squared length
     */
    public static double length2(Translation2d v) {
        double x = v.getX();
        double y = v.getY();

        return x * x + y * y;
    }

    /**
     * Retrieves the length of a 2D vector
     * @param v The vector
     * @return The length
     */
    public static double length(Translation2d v) {
        double length2 = length2(v);
        return Math.sqrt(length2);
    }

    /**
     * Normalizes a vector
     * @param v The vector to normalize
     * @return The normalized vector
     */
    public static Translation2d normalize(Translation2d v) {
        return v.div(length(v));
    }

    /**
     * Calculates the dot product of two vectors
     * @param a First vector
     * @param b Second vector
     * @return Dot product
     */
    public static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }
}
