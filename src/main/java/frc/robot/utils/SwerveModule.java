package frc.robot.utils;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX _driveMotor;
    private final TalonFX _rotationMotor;

    private final PIDController _driveController;
    private final PIDController _rotationController;

    private final CANcoder _encoder;

    public SwerveModule(int driveMotorId, int rotationMotorId, int encoderId, double angleOffset, double driveP, double rotationP) {
        _driveMotor = new TalonFX(driveMotorId);
        _rotationMotor = new TalonFX(rotationMotorId);

        // new stuff because CTRE update
        MagnetSensorConfigs encoderConfig = new MagnetSensorConfigs();
        encoderConfig.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        encoderConfig.MagnetOffset = angleOffset;

        _encoder = new CANcoder(encoderId);
        _encoder.getConfigurator().apply(encoderConfig);

        _driveController = new PIDController(driveP, 0, 0);

        _rotationController = new PIDController(rotationP, 0, 0);
        _rotationController.enableContinuousInput(-180, 180);

        TalonFXConfig.configureFalcon(_driveMotor);
        TalonFXConfig.configureFalcon(_rotationMotor);
    }

    public void drive(double speed) {
        _driveMotor.set(speed);
    }

    public void rotate(double speed) {
        _rotationMotor.set(speed);
    }

    public double getDriveVelocity() {
        double talon_rps = (_driveMotor.getRotorVelocity().getValueAsDouble() / 2048) * 10; // ctre update
        double wheel_circumference = 2 * Math.PI * Constants.Physical.SWERVE_DRIVE_WHEEL_RADIUS;

        // return the speed of the drive wheel itself (talon rps times gear ratio time wheel size) in m/s
        return (talon_rps / Constants.Physical.SWERVE_DRIVE_GEAR_RATIO) * wheel_circumference;
    }

    // TODO: make this actually work
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition();
    }

    public double getAngle() {
        return _encoder.getAbsolutePosition().getValueAsDouble() * 2 * 180; // ctre update
    }

    public void setState(SwerveModuleState state) {
        // TODO: TEST THAT THIS WORKS
        state = SwerveModuleState.optimize(state, new Rotation2d(Math.toRadians(getAngle())));
        double speed = MathUtil.clamp(state.speedMetersPerSecond, -Constants.Speeds.SWERVE_DRIVE_MAX_SPEED, Constants.Speeds.SWERVE_DRIVE_MAX_SPEED);

        double rotation_volts = -MathUtil.clamp(_rotationController.calculate(getAngle(), state.angle.getDegrees()), -1.5, 1.5);

        double drive_pid = _driveController.calculate(getDriveVelocity(), speed);
        double drive_output = (speed / Constants.Speeds.SWERVE_DRIVE_MAX_SPEED) * Constants.Speeds.SWERVE_DRIVE_COEFF;
        drive_output += drive_pid;

        rotate(rotation_volts / RobotController.getBatteryVoltage());
        drive(drive_output);
    }
}