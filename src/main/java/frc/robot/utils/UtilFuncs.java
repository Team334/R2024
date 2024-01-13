package frc.robot.utils;

/** Any utility functions for anything. */
public final class UtilFuncs {
    /** Applies deadband to a certain value. */
    public static double ApplyDeadband(double val, double deadband) {
        if (Math.abs(val) > deadband) {
            return val;
        }

        return 0;
    }
}
