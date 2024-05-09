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
        return parallel(wrist.goTo(WristPosition.INTAKE, false), intake.intake()).withName("Routines.intake()");
    }

    /**
     * Shoots the configured short distance.
     */
    public static Command shootShort() {
        return sequence(wrist.goTo(WristPosition.SHOOT_SHORT), parallel(intake.shoot(IntakeSpeed.SHOOT_SHORT), wrist.maintainPosition()))
            .withName("Routines.shootShort()");
    }

    /**
     * Shoots the configured medium distance.
     */
    public static Command shootMedium() {
        return sequence(wrist.goTo(WristPosition.SHOOT_MEDIUM), parallel(intake.shoot(IntakeSpeed.SHOOT_MEDIUM), wrist.maintainPosition()))
            .withName("Routines.shootMedium()");
    }

    /**
     * Shoots the configured far distance.
     */
    public static Command shootFar() {
        return sequence(wrist.goTo(WristPosition.SHOOT_FAR), parallel(intake.shoot(IntakeSpeed.SHOOT_FAR), wrist.maintainPosition()))
            .withName("Routines.shootFar()");
    }

    /**
     * Should be ran while disabled, and cancelled when enabled.
     * Calls {@code onDisable()} for all subsystems.
     */
    public static Command onDisable() {
        return parallel(intake.onDisable(), wrist.onDisable()).withName("Routines.onDisable()");
    }
}
