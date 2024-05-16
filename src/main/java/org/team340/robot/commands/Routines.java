package org.team340.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static org.team340.robot.RobotContainer.*;

import edu.wpi.first.wpilibj2.command.Command;
import org.team340.robot.Constants.IntakeConstants.IntakeSpeed;
import org.team340.robot.Constants.WristConstants.WristPosition;

/**
 * This class is used to declare commands that require multiple subsystems.
 */
public class Routines {

    private Routines() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Moves the wrist and runs the rollers to intake.
     */
    public static Command intake() {
        return parallel(wrist.goTo(WristPosition.INTAKE, false), intake.intake(), lights.intaking()).withName("Routines.intake()");
    }

    /**
     * Shoots at a specified speed and position.
     */
    private static Command shoot(IntakeSpeed shootSpeed, WristPosition wristPosition) {
        return parallel(
            sequence(
                parallel(intake.prepShoot(shootSpeed), wrist.goTo(wristPosition)),
                parallel(intake.shoot(shootSpeed), wrist.maintainPosition())
            ),
            lights.flames()
        )
            .withName("Routines.shoot(" + shootSpeed.name() + ")");
    }

    /**
     * Shoots the configured short distance.
     */
    public static Command shootShort() {
        return shoot(IntakeSpeed.SHOOT_SHORT, WristPosition.SHOOT_SHORT);
    }

    /**
     * Shoots the configured medium distance.
     */
    public static Command shootMedium() {
        return shoot(IntakeSpeed.SHOOT_MEDIUM, WristPosition.SHOOT_MEDIUM);
    }

    /**
     * Shoots the configured far distance.
     */
    public static Command shootFar() {
        return shoot(IntakeSpeed.SHOOT_FAR, WristPosition.SHOOT_FAR);
    }

    /**
     * Should be ran while disabled, and cancelled when enabled.
     * Calls {@code onDisable()} for all subsystems.
     */
    public static Command onDisable() {
        return parallel(intake.onDisable(), wrist.onDisable()).withName("Routines.onDisable()");
    }
}
