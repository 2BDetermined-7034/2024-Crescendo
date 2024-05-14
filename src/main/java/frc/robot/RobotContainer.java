// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.climb.ClimbDownCommand;
import frc.robot.commands.climb.ClimbUpCommand;
import frc.robot.commands.intake.BetterIntakeCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.shooter.*;
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

    public final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/kraken"));

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final PS5Controller operatorController = new PS5Controller(1);
    final PS5Controller driverController = new PS5Controller(0);

//    public static final Photonvision photonvision = new Photonvision(Constants.Vision.shooterMonoCam, Constants.Vision.lowerRobotToCamera);


    public  final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public  final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    public  final LaserCANSensor intakeLaser = new LaserCANSensor(1);
    public  final LaserCANSensor shooterLaser = new LaserCANSensor(0);
    private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem, shooterSubsystem, intakeLaser, shooterLaser);
    private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem, swerve);
    private final SourceIntake sourceIntake = new SourceIntake(shooterSubsystem);
    private final ShooterAmpCommand ampCommand = new ShooterAmpCommand(shooterSubsystem);
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final ClimbUpCommand climbUpCommand = new ClimbUpCommand(climbSubsystem);
    private final ClimbDownCommand climbDownCommand = new ClimbDownCommand(climbSubsystem);

    private final AutoFactory autoFactory;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        autoFactory = new AutoFactory(this);

        registerPathplannerCommands();
        // Configure the trigger bindings
        configureBindings();

        //Add Auto Options
        autoChooser = new SendableChooser<>();
        autoChooser.addOption("Two piece mid W2", autoFactory.getAutonomousCommand("Mid Two Piece"));
        autoChooser.addOption("Two piece amp W3", autoFactory.getAutonomousCommand("Amp Two Piece"));
        autoChooser.addOption("Two piece source W2", autoFactory.getAutonomousCommand("Source Two Piece"));
        autoChooser.addOption("Two piece source out W2", autoFactory.getAutonomousCommand("Source Two Piece W3 Out"));
        autoChooser.addOption("No Constant Amp Two Piece W1", autoFactory.getAutonomousCommand("NoConstant Amp Two Piece"));
        autoChooser.addOption("No Constant Mid Two Piece W2", autoFactory.getAutonomousCommand("NoConstant Mid Two Piece"));
        autoChooser.addOption("No Constant Mid Amp Piece W1,C1", autoFactory.getAutonomousCommand("NoConstant Amp Three Piece W1, C1"));
        autoChooser.addOption("No Constant Mid Mid Piece W2,C2", autoFactory.getAutonomousCommand("NoConstant Mid Three Piece W2,C2"));
        autoChooser.addOption("No Constant Source C5", autoFactory.getAutonomousCommand("NoConstant Source C5"));
        autoChooser.addOption("Three piece mid W2,C2", autoFactory.getAutonomousCommand("Mid Three Piece W2,C2"));
        autoChooser.addOption("Three piece amp W1,C1", autoFactory.getAutonomousCommand("Amp Three Piece W1,C1"));
        autoChooser.addOption("Exit", autoFactory.getAutonomousCommand("Exit"));







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
        new Trigger(driverController::getCrossButton).toggleOnTrue(new ShooterCommandToAngle(shooterSubsystem, 50));

        //new Trigger(driverController::getR1Button).whileTrue(climbUpCommand);
        //new Trigger(driverController::getL1Button).whileTrue(climbDownCommand);
        new Trigger(driverController::getSquareButton).whileTrue(sourceIntake);
//        new Trigger(driverController::getCrossButton).toggleOnTrue(new ShooterCommandToAngle(shooterSubsystem, 48));
//        new Trigger(driverController::getCrossButton).onFalse(new InstantCommand(() -> intakeSubsystem.run(-0, -0)));

        new Trigger(operatorController::getCircleButton).onTrue(new ShooterReset(shooterSubsystem));
        //new Trigger(operatorController::getCrossButton).toggleOnTrue(shooterCommand);
        new Trigger(operatorController::getSquareButton).whileTrue(sourceIntake);
        new Trigger(operatorController::getTriangleButton).toggleOnTrue(ampCommand);
        new Trigger(operatorController::getL1Button).whileTrue(intakeCommand);
        new Trigger(operatorController::getR1Button).whileTrue(intakeCommand);
        new Trigger(() -> operatorController.getL2Axis() > 0.5).whileTrue(new ClimbDownCommand(climbSubsystem));
        new Trigger(() -> operatorController.getR2Axis() > 0.5).whileTrue(new ClimbUpCommand(climbSubsystem));
        new Trigger(operatorController::getCreateButton).toggleOnTrue(new ShooterCommandToAngle(shooterSubsystem, -20));
        new Trigger(operatorController::getOptionsButton).toggleOnTrue(new TempShooterCommand(shooterSubsystem, -20));

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
        NamedCommands.registerCommand("Stall Intake", autoFactory.stallIntake());
        NamedCommands.registerCommand("Stall Intake Slow", autoFactory.stallIntakeSlow());

        NamedCommands.registerCommand("Better Intake", new BetterIntakeCommand(intakeSubsystem, shooterSubsystem, shooterLaser));


        NamedCommands.registerCommand("Shoot Note", autoFactory.shootNote());
        NamedCommands.registerCommand("Constant Shooter", autoFactory.constantShooter());
        NamedCommands.registerCommand("Instant Constant Shooter", autoFactory.constantShooterInstant());
//        NamedCommands.registerCommand("Constant Shooter Hardstop", autoFactory.constantShooterHardStop());
        NamedCommands.registerCommand("Angle Shooter Hardstop", autoFactory.angleShooterHardStop());
        NamedCommands.registerCommand("Rotate to speaker", new RotateToTag(swerve));

        NamedCommands.registerCommand("Shoot Note Shortcut", autoFactory.shootNoteShortcut());
        NamedCommands.registerCommand("Alignment Shortcut 36", autoFactory.shooterAlign(36));
        NamedCommands.registerCommand("Angle Shooter 36 degrees", autoFactory.angleShooter(36));

        NamedCommands.registerCommand("Alignment Shortcut 27", autoFactory.shooterAlign(27));
        NamedCommands.registerCommand("Angle Shooter 27 degrees", autoFactory.angleShooter(27));

        NamedCommands.registerCommand("Alignment Shortcut 29", autoFactory.shooterAlign(29));
        NamedCommands.registerCommand("Angle Shooter 29 degrees", autoFactory.angleShooter(29));

        NamedCommands.registerCommand("Alignment Shortcut 23", autoFactory.shooterAlign(23));

        NamedCommands.registerCommand("Alignment Shortcut 50", autoFactory.shooterAlign(50));
        NamedCommands.registerCommand("Angle Shooter 50", autoFactory.angleShooter(50));


        NamedCommands.registerCommand("Angle Shooter 23 degrees", autoFactory.angleShooter(23));

        NamedCommands.registerCommand("Alignment Shortcut 24", autoFactory.shooterAlign(24));
        NamedCommands.registerCommand("Angle Shooter 24 degrees", autoFactory.angleShooter(24));

        NamedCommands.registerCommand("Alignment Shortcut 25.5", autoFactory.shooterAlign(25.5));
        NamedCommands.registerCommand("Angle Shooter 25.5 degrees", autoFactory.angleShooter(25.5));

        NamedCommands.registerCommand("Alignment Shortcut 26", autoFactory.shooterAlign(26));
        NamedCommands.registerCommand("Angle Shooter 26 degrees", autoFactory.angleShooter(26));

        NamedCommands.registerCommand("Alignment Shortcut 20", autoFactory.shooterAlign(20));
        NamedCommands.registerCommand("Angle Shooter 20 degrees", autoFactory.angleShooter(20));

        NamedCommands.registerCommand("Alignment Shortcut 22", autoFactory.shooterAlign(22));
        NamedCommands.registerCommand("Angle Shooter 22 degrees", autoFactory.angleShooter(22));

        NamedCommands.registerCommand("Shoot Note Routine 21.5 Align", autoFactory.shootNoteRoutine(21.5, true));


        NamedCommands.registerCommand("Shoot Note Routine 22 Align", autoFactory.shootNoteRoutine(22, true));

        NamedCommands.registerCommand("Shoot Note Routine 27", autoFactory.shootNoteRoutine(27, false));
        NamedCommands.registerCommand("Shoot Note Routine 27 Align", autoFactory.shootNoteRoutine(27, true));


        NamedCommands.registerCommand("Shoot Note Routine 50", autoFactory.shootNoteRoutine(50, false));

        NamedCommands.registerCommand("Shoot Note Routine 50 Align", autoFactory.shootNoteRoutine(50, true));

        NamedCommands.registerCommand("Shoot Note Routine 23", autoFactory.shootNoteRoutine(23, false));
        NamedCommands.registerCommand("Shoot Note Routine 23 Align", autoFactory.shootNoteRoutine(23, true));

        NamedCommands.registerCommand("Shoot Note Routine 45 Align", autoFactory.shootNoteRoutine(45, true));

        NamedCommands.registerCommand("Shoot Note Routine 25.5", autoFactory.shootNoteRoutine(25.5, false));
        NamedCommands.registerCommand("Shoot Note Routine 25.5 Align", autoFactory.shootNoteRoutine(25.5, true));

        NamedCommands.registerCommand("Source Intake", sourceIntake);

    }

    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }


}
