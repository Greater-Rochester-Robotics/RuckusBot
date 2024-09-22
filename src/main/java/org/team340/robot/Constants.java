package org.team340.robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.ExternalFollower;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.team340.lib.controller.Controller2Config;
import org.team340.lib.swerve.config.SwerveConfig;
import org.team340.lib.swerve.config.SwerveModuleConfig;
import org.team340.lib.swerve.hardware.motors.SwerveMotor;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.rev.SparkAbsoluteEncoderConfig;
import org.team340.lib.util.config.rev.SparkMaxConfig;
import org.team340.lib.util.config.rev.SparkMaxConfig.Frame;
import org.team340.lib.util.config.rev.SparkPIDControllerConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double PERIOD = 0.020;
    public static final double TELEMETRY_PERIOD = 0.020;
    public static final double POWER_USAGE_PERIOD = 0.020;
    public static final double VOLTAGE = 12.0;
    public static final double FIELD_LENGTH = 16.541;
    public static final double FIELD_WIDTH = 8.211;

    /**
     * Driver and co-driver controller constants.
     */
    public static final class ControllerConstants {

        public static final double DRIVE_EXP = 1.0;
        public static final double DRIVE_MULTIPLIER = 0.9;
        public static final double DRIVE_MULTIPLIER_MODIFIED = 1.0;

        public static final double DRIVE_ROT_EXP = 2.0;
        public static final double DRIVE_ROT_MULTIPLIER = 0.4;

        public static final Controller2Config DRIVER = new Controller2Config()
            .setLabel("Driver")
            .setPort(0)
            .setJoystickDeadband(0.05)
            .setJoystickThreshold(0.5)
            .setTriggerDeadband(0.05)
            .setTriggerThreshold(0.1);
    }

    /**
     * Map of hardware device IDs.
     */
    public static final class RobotMap {

        public static final int FRONT_LEFT_MOVE = 2;
        public static final int FRONT_LEFT_TURN = 3;
        public static final int BACK_LEFT_MOVE = 4;
        public static final int BACK_LEFT_TURN = 5;
        public static final int BACK_RIGHT_MOVE = 6;
        public static final int BACK_RIGHT_TURN = 7;
        public static final int FRONT_RIGHT_MOVE = 8;
        public static final int FRONT_RIGHT_TURN = 9;
        public static final int FRONT_LEFT_ENCODER = 10;
        public static final int BACK_LEFT_ENCODER = 11;
        public static final int BACK_RIGHT_ENCODER = 12;
        public static final int FRONT_RIGHT_ENCODER = 13;

        public static final int WRIST_MOTOR = 20;

        public static final int INTAKE_UPPER_MOTOR = 30;
        public static final int INTAKE_LOWER_MOTOR = 31;
        public static final int INTAKE_INNER_MOTOR = 32;

        public static final int LIGHTS = 9;
    }

    /**
     * Constants for the intake subsystem.
     */
    public static final class IntakeConstants {

        // Speeds
        public enum IntakeSpeed {
            HOLD_INNER(0.05),
            INTAKE_OUTER(0.4),
            INTAKE_INNER(0.3),
            SHOOT_SHORT(-0.25, 0.6),
            SHOOT_MEDIUM(-0.3, 0.7),
            SHOOT_FAR(-1.0, 2.0),
            SHOOT_INNER(-1.0);

            public final double value;
            public final double spinTime;

            private IntakeSpeed(double value) {
                this(value, 0.0);
            }

            private IntakeSpeed(double value, double spinTime) {
                this.value = value;
                this.spinTime = spinTime;
            }
        }

        // Hardware Configs
        public static final class Configs {

            private static final SparkMaxConfig MOTOR_BASE = new SparkMaxConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(30)
                .setIdleMode(IdleMode.kCoast)
                .setPeriodicFramePeriod(Frame.S0, 20)
                .setPeriodicFramePeriod(Frame.S1, 20)
                .setPeriodicFramePeriod(Frame.S2, 20)
                .setPeriodicFramePeriod(Frame.S3, 10000)
                .setPeriodicFramePeriod(Frame.S4, 10000)
                .setPeriodicFramePeriod(Frame.S5, 10000)
                .setPeriodicFramePeriod(Frame.S6, 10000);

            public static final SparkMaxConfig UPPER_MOTOR = MOTOR_BASE.clone().setInverted(false);
            public static final SparkMaxConfig LOWER_MOTOR = MOTOR_BASE
                .clone()
                .follow(ExternalFollower.kFollowerSpark, RobotMap.INTAKE_UPPER_MOTOR, true);
            public static final SparkMaxConfig INNER_MOTOR = MOTOR_BASE.clone().setInverted(false);
        }
    }

    /**
     * Constants for the lights subsystem.
     */
    public static final class LightsConstants {

        public static final int LENGTH = 56;
    }

    /**
     * Constants for the swerve subsystem.
     */
    public static final class SwerveConstants {

        private static final SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig()
            .setLabel("Front Left")
            .useCANcoder(RobotMap.FRONT_LEFT_ENCODER, -3.8534, false)
            .setPosition(0.288925, 0.288925)
            .setMoveMotor(RobotMap.FRONT_LEFT_MOVE, true, false)
            .setTurnMotor(RobotMap.FRONT_LEFT_TURN, false, true);

        private static final SwerveModuleConfig BACK_LEFT = new SwerveModuleConfig()
            .setLabel("Back Left")
            .useCANcoder(RobotMap.BACK_LEFT_ENCODER, -3.1124, false)
            .setPosition(-0.288925, 0.288925)
            .setMoveMotor(RobotMap.BACK_LEFT_MOVE, true, false)
            .setTurnMotor(RobotMap.BACK_LEFT_TURN, false, true);

        private static final SwerveModuleConfig BACK_RIGHT = new SwerveModuleConfig()
            .setLabel("Back Right")
            .useCANcoder(RobotMap.BACK_RIGHT_ENCODER, -6.2234, false)
            .setPosition(-0.288925, -0.288925)
            .setMoveMotor(RobotMap.BACK_RIGHT_MOVE, true, false)
            .setTurnMotor(RobotMap.BACK_RIGHT_TURN, false, true);

        private static final SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig()
            .setLabel("Front Right")
            .useCANcoder(RobotMap.FRONT_RIGHT_ENCODER, -1.4365, false)
            .setPosition(0.288925, -0.288925)
            .setMoveMotor(RobotMap.FRONT_RIGHT_MOVE, true, false)
            .setTurnMotor(RobotMap.FRONT_RIGHT_TURN, false, true);

        public static final SwerveConfig CONFIG = new SwerveConfig()
            .useADIS16470(IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, Port.kOnboardCS0, CalibrationTime._4s)
            .setPeriod(PERIOD)
            .setMovePID(0.2, 0.0, 0.0, 0.0)
            .setMoveFF(0.05, 2.35, 0.0)
            .setTurnPID(0.65, 0.001, 3.0, 0.01)
            .setRampRate(0.03, 0.03)
            .setMotorTypes(SwerveMotor.Type.SPARK_MAX_BRUSHLESS, SwerveMotor.Type.SPARK_MAX_BRUSHLESS)
            .setMaxSpeeds(5.3, 12.3)
            .setRatelimits(8.0, 10.0, 40.0)
            .setTrajectoryConstraints(4.0, 8.0)
            .setPowerProperties(VOLTAGE, 40.0, 40.0)
            .setMechanicalProperties(7.5, 10.0, 4.0)
            .setDiscretizationLookahead(PERIOD)
            .setOdometryPeriod(PERIOD)
            .setOdometryStd(0.003, 0.003, 0.0012)
            .setVisionStd(0.0, 0.0, 0.0)
            .setSysIdConfig(new SysIdRoutine.Config(Volts.of(1.0).per(Seconds.of(0.4)), Volts.of(7.0), Seconds.of(5.5)))
            .setFieldSize(FIELD_LENGTH, FIELD_WIDTH)
            .addModule(FRONT_LEFT)
            .addModule(BACK_LEFT)
            .addModule(BACK_RIGHT)
            .addModule(FRONT_RIGHT);
    }

    /**
     * Constants for the wrist subsystem.
     */
    public static final class WristConstants {

        // Limits
        public static final double MIN_POS = Math.toRadians(20.0);
        public static final double MAX_POS = Math.toRadians(140.0);

        // Positions
        public enum WristPosition {
            INTAKE(Math.toRadians(132.0)),
            SAFE(Math.toRadians(25.0)),
            SHOOT_SHORT(Math.toRadians(80.0)),
            SHOOT_MEDIUM(Math.toRadians(55.0)),
            SHOOT_FAR(Math.toRadians(45.0));

            public final double value;

            private WristPosition(double value) {
                this.value = value;
            }
        }

        // Misc
        public static final double CLOSED_LOOP_ERR = Math.toRadians(4.0);

        // Hardware Configs
        public static final class Configs {

            // Encoder Conversion Factor
            private static final double ENC_FACTOR = Math2.TWO_PI;

            public static final SparkMaxConfig MOTOR = new SparkMaxConfig()
                .clearFaults()
                .restoreFactoryDefaults()
                .enableVoltageCompensation(VOLTAGE)
                .setSmartCurrentLimit(30)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(true)
                .setClosedLoopRampRate(0.3)
                .setOpenLoopRampRate(0.8)
                .setPeriodicFramePeriod(Frame.S0, 20)
                .setPeriodicFramePeriod(Frame.S1, 20)
                .setPeriodicFramePeriod(Frame.S2, 20)
                .setPeriodicFramePeriod(Frame.S3, 10000)
                .setPeriodicFramePeriod(Frame.S4, 10000)
                .setPeriodicFramePeriod(Frame.S5, 20)
                .setPeriodicFramePeriod(Frame.S6, 20);

            public static final SparkAbsoluteEncoderConfig ENCODER = new SparkAbsoluteEncoderConfig()
                .setPositionConversionFactor(ENC_FACTOR)
                .setVelocityConversionFactor(ENC_FACTOR / 60)
                .setInverted(true)
                .setZeroOffset(0.0);

            public static final SparkPIDControllerConfig PID = new SparkPIDControllerConfig()
                .setPID(1.85, 0.0, 0.3)
                .setIZone(0.0)
                .setPositionPIDWrappingEnabled(false);
        }
    }
}
