package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

/** Contains controllers for robot and additional functions. */
public class RobotCtrl {
    public final CommandPS4Controller driveController = new CommandPS4Controller(Constants.Ports.DRIVER_CONTROLLER);

    private final SlewRateLimiter _driveFilterLeftX = new SlewRateLimiter(4);
    private final SlewRateLimiter _driveFilterLeftY = new SlewRateLimiter(4);

    private final SlewRateLimiter _driveFilterRightX = new SlewRateLimiter(4);
    private final SlewRateLimiter _driveFilterRightY = new SlewRateLimiter(4);

    public double driveLeftX() {
        return _driveFilterLeftX.calculate(driveController.getLeftX());
    }

    public double driveLeftY() {
        return _driveFilterLeftY.calculate(driveController.getLeftY());
    }

    public double driveRightX() {
        return _driveFilterRightX.calculate(driveController.getRightX());
    }

    public double driveRightY() {
        return _driveFilterRightY.calculate(driveController.getRightY());
    }

    /** Applies deadband to a certain value. */
    public static double ApplyDeadband(double val, double deadband) {
        if (Math.abs(val) > deadband) {
            return val;
        }

        return 0;
    }
}