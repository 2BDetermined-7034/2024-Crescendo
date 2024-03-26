// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Auto.AutoFactory;
import frc.robot.commands.climb.ClimbDownCommand;
import frc.robot.commands.climb.ClimbUpCommand;
import frc.robot.commands.intake.AutoIntakeCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.*;
import frc.robot.commands.swervedrive.drivebase.RotateToAnyTag;
import frc.robot.commands.swervedrive.drivebase.RotateToTag;
import frc.robot.commands.swervedrive.drivebase.TeleopDrive;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.sensors.LaserCANSensor;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Photonvision;

import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final SendableChooser<Command> autoChooser;

    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/kraken"));

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final PS5Controller operatorController = new PS5Controller(1);
    final PS5Controller driverController = new PS5Controller(0);

//    public static final Photonvision photonvision = new Photonvision(Constants.Vision.shooterMonoCam, Constants.Vision.shooterCamToRobotTransfrom);


    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final LaserCANSensor shooterLaser = new LaserCANSensor(0);
    private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, shooterSubsystem, shooterLaser);
    private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem, swerve);
    private final SourceIntake sourceIntake = new SourceIntake(shooterSubsystem);
    private final ShooterAmpCommand ampCommand = new ShooterAmpCommand(shooterSubsystem);
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final ClimbUpCommand climbUpCommand = new ClimbUpCommand(climbSubsystem);
    private final ClimbDownCommand climbDownCommand = new ClimbDownCommand(climbSubsystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        registerPathplannerCommands();
        // Configure the trigger bindings
        configureBindings();

        //Add Auto Options
        autoChooser = new SendableChooser<>();

        autoChooser.addOption("Choreo Mid Two Piece", AutoFactory.getAutonomousCommand("Mid Two Piece"));
        autoChooser.addOption("Choreo Amp Two Piece", AutoFactory.getAutonomousCommand("Amp Two Piece"));
        autoChooser.addOption("Choreo Source Two Piece", AutoFactory.getAutonomousCommand("Source Two Piece"));
        autoChooser.addOption("Choreo Amp Three Piece", AutoFactory.getAutonomousCommand("Amp Three Piece"));
        autoChooser.addOption("Choreo Mid Three Piece", AutoFactory.getAutonomousCommand("Mid Three Piece"));
        autoChooser.addOption("Choreo Source Three Piece", AutoFactory.getAutonomousCommand("Source Three Piece"));
        autoChooser.addOption("Choreo Mid Four Piece", AutoFactory.getAutonomousCommand("Mid Four Piece"));
        autoChooser.addOption("Choreo Amp Four Piece", AutoFactory.getAutonomousCommand("Amp Four Piece"));
        autoChooser.addOption("Choreo Source Four Piece", AutoFactory.getAutonomousCommand("Source Four Piece"));






        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));

        SmartDashboard.putData("Auto Chooser", autoChooser);

		/*
		This Command uses both x and y from right analogue stick to control desired angle instead of angular rotation
		 */
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        Command driveFieldOrientedDirectAngle = swerve.driveCommand(
                () -> -MathUtil.applyDeadband(driverController.getLeftY(), Constants.DriverConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(driverController.getLeftX(), Constants.DriverConstants.LEFT_X_DEADBAND),
                () -> -driverController.getRightX(),
                () -> -driverController.getRightY());

        Command driveFieldOrientedTeleop = new TeleopDrive(swerve,
                () -> -MathUtil.applyDeadband(driverController.getLeftY(), Constants.DriverConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(driverController.getLeftX(), Constants.DriverConstants.LEFT_X_DEADBAND),
                () -> -MathUtil.applyDeadband(driverController.getRightX(), 0.25),
                () -> true);

        swerve.setDefaultCommand(driveFieldOrientedTeleop);

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
//        new Trigger(driverController::getTriangleButton).toggleOnTrue(intakeCommand);
        new Trigger(driverController::getTriangleButton).toggleOnTrue(new ShooterPodiumCommand(shooterSubsystem));
        new Trigger(driverController::getCircleButton).toggleOnTrue(shooterCommand);
        new Trigger(driverController::getL1Button).toggleOnTrue(new RotateToTag(swerve));


        //new Trigger(driverController::getR1Button).whileTrue(climbUpCommand);
        //new Trigger(driverController::getL1Button).whileTrue(climbDownCommand);
        new Trigger(driverController::getSquareButton).whileTrue(sourceIntake);
        new Trigger(driverController::getCrossButton).whileTrue(intakeCommand);
//        new Trigger(driverController::getCrossButton).onFalse(new InstantCommand(() -> intakeSubsystem.run(-0, -0)));

        new Trigger(operatorController::getCircleButton).onTrue(new ShooterReset(shooterSubsystem));
        //new Trigger(operatorController::getCrossButton).toggleOnTrue(shooterCommand);
        new Trigger(operatorController::getSquareButton).whileTrue(sourceIntake);
        new Trigger(operatorController::getTriangleButton).toggleOnTrue(ampCommand);
        new Trigger(operatorController::getL1Button).whileTrue(intakeCommand);
        new Trigger(operatorController::getR1Button).whileTrue(intakeCommand);
        new Trigger(() -> operatorController.getL2Axis() > 0.5).whileTrue(new ClimbDownCommand(climbSubsystem));
        new Trigger(() -> operatorController.getR2Axis() > 0.5).whileTrue(new ClimbUpCommand(climbSubsystem));
        new Trigger(operatorController::getOptionsButton).toggleOnTrue(new ShooterCommandToAngle(shooterSubsystem, -20));

    }



    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public void setDriveMode() {
        //drivebase.setDefaultCommand();
    }
    public void registerPathplannerCommands() {
        NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> {intakeSubsystem.run(0, 0); shooterSubsystem.setLaunchTalon(0);}));
        NamedCommands.registerCommand("Shoot Note", shooterCommand);
        NamedCommands.registerCommand("Auto Intake", new AutoIntakeCommand(intakeSubsystem, shooterSubsystem));
        NamedCommands.registerCommand("Intake Command", intakeCommand);

        NamedCommands.registerCommand("Rotate to Tag", new RotateToTag(swerve));
    }

    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }
}
