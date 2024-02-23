// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.vision.PhotonVision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController operatorController = new CommandXboxController(1);
    final PS5Controller driverController = new PS5Controller(0);

    // Vision
    final static PhotonVision shootercam = new PhotonVision("greencasecam", new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));
    final static PhotonVision intakecam = new PhotonVision("whitecasecam", new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));


    SendableChooser autoSelector;
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        autoSelector = new SendableChooser<>();
        autoSelector.setDefaultOption("Do Nothing", new WaitCommand(1));

        SmartDashboard.putData(autoSelector);

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        Command driveFieldOrientedDirectAngle = swerve.driveCommand(
                () -> -MathUtil.applyDeadband(operatorController.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(operatorController.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> -operatorController.getRightX(),
                () -> -operatorController.getRightY());

        swerve.setDefaultCommand(driveFieldOrientedDirectAngle);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(driverController::getOptionsButton).onTrue(Commands.runOnce(swerve::zeroGyro));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return swerve.getAutonomousCommand("New Auto");
    }

    public void setDriveMode() {
        //drivebase.setDefaultCommand();
    }

    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }
}
