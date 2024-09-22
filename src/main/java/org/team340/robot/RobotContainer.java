package org.team340.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import org.team340.lib.controller.Controller2;
import org.team340.lib.util.Math2;
import org.team340.lib.util.config.rev.RevConfigRegistry;
import org.team340.robot.Constants.ControllerConstants;
import org.team340.robot.Constants.WristConstants.WristPosition;
import org.team340.robot.commands.Routines;
import org.team340.robot.subsystems.Intake;
import org.team340.robot.subsystems.Lights;
import org.team340.robot.subsystems.Lights.DrivingMode;
import org.team340.robot.subsystems.Swerve;
import org.team340.robot.subsystems.Wrist;

/**
 * This class is used to declare subsystems, commands, and trigger mappings.
 */
public final class RobotContainer {

    private RobotContainer() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    private static Controller2 driver;

    public static Intake intake;
    public static Lights lights;
    public static Swerve swerve;
    public static Wrist wrist;

    /**
     * Entry to initializing subsystems and command execution.
     */
    public static void init() {
        // Initialize controllers.
        driver = new Controller2(ControllerConstants.DRIVER);

        // Add controllers to the dashboard.
        driver.addToDashboard();

        // Initialize subsystems.
        intake = new Intake();
        lights = new Lights();
        swerve = new Swerve();
        wrist = new Wrist();

        // Add subsystems to the dashboard.
        intake.addToDashboard();
        lights.addToDashboard();
        swerve.addToDashboard();
        wrist.addToDashboard();

        // Complete REV hardware initialization.
        RevConfigRegistry.burnFlash();
        RevConfigRegistry.printError();

        // Configure bindings and autos.
        configBindings();
    }

    /**
     * This method should be used to declare triggers (created with an
     * arbitrary predicate or from controllers) and their bindings.
     */
    private static void configBindings() {
        // Set default commands.
        intake.setDefaultCommand(intake.hold());
        lights.setDefaultCommand(lights.idle());
        swerve.setDefaultCommand(
            parallel(
                swerve.drive(RobotContainer::getDriveX, RobotContainer::getDriveY, RobotContainer::getDriveRotate, true),
                lights.setDrivingMode(DrivingMode.FIELD_RELATIVE)
            )
        );

        Routines.onDisable().schedule();
        RobotModeTriggers.disabled().onTrue(Routines.onDisable());

        /**
         * Driver bindings.
         */

        // A => Intake (Hold)
        driver.a().whileTrue(Routines.intake()).onFalse(wrist.goTo(WristPosition.SAFE));

        // B => Shoot short (Hold)
        driver.b().whileTrue(Routines.shootShort()).onFalse(wrist.goTo(WristPosition.SAFE));

        // X => Shoot medium (Hold)
        driver.x().whileTrue(Routines.shootMedium()).onFalse(wrist.goTo(WristPosition.SAFE));

        // Y => Shoot far (Hold)
        driver.y().whileTrue(Routines.shootFar()).onFalse(wrist.goTo(WristPosition.SAFE));

        // POV Left => Zero swerve
        driver.povLeft().onTrue(swerve.zeroIMU(Math2.ROTATION2D_0));

        // Back => Toggle arcade driving
        driver
            .back()
            .toggleOnTrue(
                parallel(
                    swerve.drive(RobotContainer::getDriveX, () -> 0.0, RobotContainer::getDriveYRotate, false),
                    lights.setDrivingMode(DrivingMode.ARCADE)
                )
            );

        // Start => Toggle robot relative driving
        driver
            .start()
            .toggleOnTrue(
                parallel(
                    swerve.drive(RobotContainer::getDriveX, RobotContainer::getDriveY, RobotContainer::getDriveRotate, false),
                    lights.setDrivingMode(DrivingMode.ROBOT_RELATIVE)
                )
            );
    }

    /**
     * Gets the X axis drive speed from the driver's controller.
     */
    private static double getDriveX() {
        double multiplier =
            ((driver.getHID().getLeftStickButton()) ? ControllerConstants.DRIVE_MULTIPLIER_MODIFIED : ControllerConstants.DRIVE_MULTIPLIER);
        return -driver.getLeftY(multiplier, ControllerConstants.DRIVE_EXP);
    }

    /**
     * Gets the Y axis drive speed from the driver's controller.
     */
    private static double getDriveY() {
        double multiplier =
            ((driver.getHID().getLeftStickButton()) ? ControllerConstants.DRIVE_MULTIPLIER_MODIFIED : ControllerConstants.DRIVE_MULTIPLIER);
        return -driver.getLeftX(multiplier, ControllerConstants.DRIVE_EXP);
    }

    /**
     * Gets the Y axis rotational drive speed from the driver's controller.
     */
    private static double getDriveYRotate() {
        return -driver.getLeftX(ControllerConstants.DRIVE_ROT_MULTIPLIER, ControllerConstants.DRIVE_ROT_EXP);
    }

    /**
     * Gets the rotational drive speed from the driver's controller.
     */
    private static double getDriveRotate() {
        return driver.getTriggerDifference(ControllerConstants.DRIVE_ROT_MULTIPLIER, ControllerConstants.DRIVE_ROT_EXP);
    }
}
