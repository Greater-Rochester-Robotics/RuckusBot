package org.team340.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.team340.lib.swerve.SwerveAPI;
import org.team340.lib.swerve.SwerveAPI.ForwardPerspective;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.SwerveEncoders;
import org.team340.lib.swerve.hardware.SwerveIMUs;
import org.team340.lib.swerve.hardware.SwerveMotors;
import org.team340.lib.util.GRRSubsystem;
import org.team340.robot.Constants;
import org.team340.robot.Constants.RobotMap;

@Logged
public class Swerve extends GRRSubsystem {

    private static final SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig()
        .setName("frontLeft")
        .setLocation(0.28, 0.28)
        .setMoveMotor(SwerveMotors.sparkMax(RobotMap.FL_MOVE, MotorType.kBrushless, false))
        .setTurnMotor(SwerveMotors.sparkMax(RobotMap.FL_TURN, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.FL_ENCODER, 0.3867, false));

    private static final SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig()
        .setName("frontRight")
        .setLocation(0.28, -0.28)
        .setMoveMotor(SwerveMotors.sparkMax(RobotMap.FR_MOVE, MotorType.kBrushless, false))
        .setTurnMotor(SwerveMotors.sparkMax(RobotMap.FR_TURN, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.FR_ENCODER, 0.7714, false));

    private static final SwerveModuleConfig BACK_LEFT = new SwerveModuleConfig()
        .setName("backLeft")
        .setLocation(-0.28, 0.28)
        .setMoveMotor(SwerveMotors.sparkMax(RobotMap.BL_MOVE, MotorType.kBrushless, false))
        .setTurnMotor(SwerveMotors.sparkMax(RobotMap.BL_TURN, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.BL_ENCODER, 0.5046, false));

    private static final SwerveModuleConfig BACK_RIGHT = new SwerveModuleConfig()
        .setName("backRight")
        .setLocation(-0.28, -0.28)
        .setMoveMotor(SwerveMotors.sparkMax(RobotMap.BR_MOVE, MotorType.kBrushless, false))
        .setTurnMotor(SwerveMotors.sparkMax(RobotMap.BR_TURN, MotorType.kBrushless, true))
        .setEncoder(SwerveEncoders.canCoder(RobotMap.BR_ENCODER, 0.0095, false));

    private static final SwerveConfig CONFIG = new SwerveConfig()
        .setTimings(Constants.PERIOD, Constants.PERIOD, 0.04)
        .setMovePID(0.0015, 0.0, 0.0, 0.0)
        .setMoveFF(0.05, 0.127)
        .setTurnPID(0.45, 0.0, 0.1, 0.0)
        .setBrakeMode(false, true)
        .setLimits(5.2, 13.5, 8.5, 28.0)
        .setDriverProfile(4.5, 1.0, 4.2, 2.0)
        .setPowerProperties(Constants.VOLTAGE, 80.0, 60.0)
        .setMechanicalProperties(7.5, 10.0, 0.0, Units.inchesToMeters(4.0))
        .setOdometryStd(0.1, 0.1, 0.1)
        .setIMU(SwerveIMUs.adis16470(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, Port.kOnboardCS0, CalibrationTime._4s))
        .setModules(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);

    private final SwerveAPI api;

    public Swerve() {
        api = new SwerveAPI(CONFIG);
        api.enableTunables("Swerve");
    }

    @Override
    public void periodic() {
        api.refresh();
    }

    /**
     * Drives the robot using driver input.
     * @param x The X value from the driver's joystick.
     * @param y The Y value from the driver's joystick.
     * @param angular The CCW+ angular speed to apply, from {@code [-1.0, 1.0]}.
     */
    public Command drive(Supplier<Double> x, Supplier<Double> y, Supplier<Double> angular) {
        return commandBuilder("Swerve.drive()").onExecute(() ->
            api.applyDriverInput(x.get(), y.get(), angular.get(), ForwardPerspective.OPERATOR, true, true)
        );
    }

    /**
     * Tares the rotation of the robot. Useful for
     * fixing an out of sync or drifting IMU.
     */
    public Command tareRotation() {
        return commandBuilder("Swerve.tareRotation()")
            .onInitialize(() -> api.tareRotation(ForwardPerspective.OPERATOR))
            .isFinished(true);
    }
}
