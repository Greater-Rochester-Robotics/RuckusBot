package org.team340.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Alliance;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants.LightsConstants;
import org.team340.robot.Constants.RobotMap;

/**
 * Controls the lights.
 */
public class Lights extends GRRSubsystem {

    public enum DrivingMode {
        FIELD_RELATIVE(255, 0, 10),
        ROBOT_RELATIVE(255, 80, 10),
        ARCADE(255, 100, 40);

        private final int primary;
        private final int secondary;
        private final int g;

        private DrivingMode(int primary, int secondary, int g) {
            this.primary = primary;
            this.secondary = secondary;
            this.g = g;
        }

        public int r(double intensity) {
            return (int) (intensity * (Alliance.isBlue() ? secondary : primary));
        }

        public int g(double intensity) {
            return (int) (intensity * g);
        }

        public int b(double intensity) {
            return (int) (intensity * (Alliance.isBlue() ? primary : secondary));
        }
    }

    private final AddressableLED lights;
    private final AddressableLEDBuffer buffer;
    private final Timer timer;

    private DrivingMode drivingMode = DrivingMode.FIELD_RELATIVE;

    /**
     * Create the lights subsystem.
     */
    public Lights() {
        super("Lights");
        lights = new AddressableLED(RobotMap.LIGHTS);
        buffer = new AddressableLEDBuffer(LightsConstants.LENGTH);
        timer = new Timer();

        lights.setLength(buffer.getLength());
        lights.start();
        timer.start();
    }

    @Override
    public void periodic() {
        lights.setData(buffer);
    }

    /**
     * Modifies the entire buffer to be a single color.
     * @param r Red value from {@code 0} to {@code 255}.
     * @param g Green value from {@code 0} to {@code 255}.
     * @param b Blue value from {@code 0} to {@code 255}.
     */
    private void set(int r, int g, int b) {
        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, r, g, b);
        }
    }

    /**
     * Modifies the buffer with values mirrored across the "center" of the LED strip.
     * @param i The index of the buffer to modify.
     * @param r Red value from {@code 0} to {@code 255}.
     * @param g Green value from {@code 0} to {@code 255}.
     * @param b Blue value from {@code 0} to {@code 255}.
     */
    private void setMirrored(int i, int r, int g, int b) {
        buffer.setRGB(i, r, g, b);
        buffer.setRGB(buffer.getLength() - i - 1, r, g, b);
    }

    /**
     * Displays the intaking animation.
     */
    public Command intaking() {
        return commandBuilder()
            .onExecute(() -> {
                double time = timer.get();
                for (int i = 0; i < buffer.getLength() / 2; i++) {
                    double v = (Math.cos((time * 22.0) + ((i / (buffer.getLength() / 2.0)) * Math2.TWO_PI)) + 1.0) / 2.0;
                    setMirrored(i, drivingMode.r(v), drivingMode.g(v), drivingMode.b(v));
                }
            })
            .onEnd(() -> set(0, 0, 0))
            .ignoringDisable(true)
            .withName("lights.intaking()");
    }

    /**
     * Displays the flames animation.
     */
    public Command flames() {
        int[] state = new int[buffer.getLength()];
        for (int i = 0; i < state.length; i++) {
            state[i] = 0;
        }

        return commandBuilder()
            .onExecute(() -> {
                for (int i = 0; i < buffer.getLength() / 2; i++) {
                    state[i] = (int) Math.max(0.0, state[i] - (Math2.random((0.5 + (i / (buffer.getLength() * 0.25))) * 5.0) + 28.0));
                }
                for (int i = (buffer.getLength() / 2) - 1; i >= 2; i--) {
                    state[i] = (state[i - 1] + state[i - 2] + state[i - 2]) / 3;
                }
                if (Math.random() < 0.5) {
                    int i = (int) Math2.random(5.0);
                    state[i] = (int) (state[i] + Math2.random(160.0, 255.0));
                }
                for (int i = 0; i < buffer.getLength() / 2; i++) {
                    int heat = (int) ((state[i] / 255.0) * 191.0);
                    int ramp = (heat & 63) << 2;
                    if (heat > 180) {
                        setMirrored(i, 255, 255, ramp);
                    } else if (heat > 60) {
                        setMirrored(i, 255, ramp, 0);
                    } else {
                        setMirrored(i, ramp, 0, 0);
                    }
                }
            })
            .onEnd(() -> set(0, 0, 0))
            .ignoringDisable(true)
            .withName("lights.flames()");
    }

    /**
     * Displays the idle animation.
     */
    public Command idle() {
        return commandBuilder()
            .onExecute(() -> {
                if (DriverStation.isTeleopEnabled()) {
                    double v = (((Math.cos(timer.get() * 8.5) + 1.0) / 4.0) + 0.5);
                    set(drivingMode.r(v), drivingMode.g(v), drivingMode.b(v));
                } else if (DriverStation.isDisabled()) {
                    set(drivingMode.r(1.0), drivingMode.g(1.0), drivingMode.b(1.0));
                } else {
                    set(0, 0, 0);
                }
            })
            .onEnd(() -> set(0, 0, 0))
            .ignoringDisable(true)
            .withName("lights.idle()");
    }

    /**
     * Should be called when the driving mode changes.
     * Runs instantly, modifies the default shade of the
     * lights to reflect the driving mode of the robot.
     * @param newMode The new driving mode.
     */
    public Command setDrivingMode(DrivingMode newMode) {
        return Commands.runOnce(() -> drivingMode = newMode).withName("lights.setDrivingMode(" + newMode.name() + ")");
    }
}
