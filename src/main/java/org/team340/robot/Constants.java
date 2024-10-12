package org.team340.robot;

import org.team340.lib.controller.ControllerConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 */
public final class Constants {

    public static final double PERIOD = 0.020;
    public static final double VOLTAGE = 12.0;

    public static final ControllerConfig DRIVER = new ControllerConfig()
        .setPort(0)
        .setDeadbands(0.05, 0.05)
        .setThresholds(0.5, 0.05);

    public static final ControllerConfig CO_DRIVER = new ControllerConfig()
        .setPort(1)
        .setDeadbands(0.05, 0.05)
        .setThresholds(0.5, 0.05);

    /**
     * The RobotMap class defines CAN IDs, CAN bus names, DIO/PWM/PH/PCM channel
     * IDs, and other relevant identifiers for addressing robot hardware.
     */
    public static final class RobotMap {

        public static final String CANBUS = "*";

        public static final int FL_MOVE = 2;
        public static final int FL_TURN = 3;
        public static final int FR_MOVE = 8;
        public static final int FR_TURN = 9;
        public static final int BL_MOVE = 4;
        public static final int BL_TURN = 5;
        public static final int BR_MOVE = 6;
        public static final int BR_TURN = 7;

        public static final int FL_ENCODER = 10;
        public static final int FR_ENCODER = 13;
        public static final int BL_ENCODER = 11;
        public static final int BR_ENCODER = 12;
    }
}
