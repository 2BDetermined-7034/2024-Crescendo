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
import frc.robot.commands.intake.AutoIntakeCommand;
import frc.robot.commands.intake.BetterIntakeCommand;
import frc.robot.commands.intake.BetterIntakeReverse;
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

    private final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/kraken"));

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final PS5Controller operatorController = new PS5Controller(1);
    final PS5Controller driverController = new PS5Controller(0);

    public static final Photonvision photonvision = new Photonvision(Constants.Vision.shooterMonoCam, Constants.Vision.shooterCamToRobotTransfrom);


    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);

    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
    private final LaserCANSensor laserCANSensor = new LaserCANSensor(0);
    private final ShooterCommand shooterCommand = new ShooterCommand(shooterSubsystem, swerve);
    private final SourceIntake sourceIntake = new SourceIntake(shooterSubsystem);
    private final ShooterAmpCommand ampCommand = new ShooterAmpCommand(shooterSubsystem);
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final ClimbUpCommand climbUpCommand = new ClimbUpCommand(climbSubsystem);
    private final ClimbDownCommand climbDownCommand = new ClimbDownCommand(climbSubsystem);
    private final BetterIntakeCommand betterIntakeCommand = new BetterIntakeCommand(intakeSubsystem, shooterSubsystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        registerPathplannerCommands();
        // Configure the trigger bindings
        configureBindings();

        //Add Auto Options
        autoChooser = new SendableChooser<>();
        // PathPlanner Autos
        autoChooser.addOption("One piece mid", swerve.getAutonomousCommand("1PieceMid"));
        autoChooser.addOption("One piece amp", swerve.getAutonomousCommand("1PieceAmp"));
        autoChooser.addOption("One piece source", swerve.getAutonomousCommand("1PieceSource"));
        autoChooser.addOption("Two piece mid", swerve.getAutonomousCommand("2PieceMid"));
        autoChooser.addOption("Two piece amp", swerve.getAutonomousCommand("2PieceAmp"));
        autoChooser.addOption("Two piece mid podium shot", swerve.getAutonomousCommand("2PieceMidPodiumShot"));
        autoChooser.addOption("Three piece mid", swerve.getAutonomousCommand("Choreo3PieceTheoretical"));
        autoChooser.addOption("Three piece mid center", swerve.getAutonomousCommand("Copy of Choreo3PieceTheoretical"));
        autoChooser.addOption("Four piece mid", swerve.getAutonomousCommand("Choreo4PieceTheoretical"));
        autoChooser.addOption("Four piece mid test", swerve.getAutonomousCommand("Choreo4PieceTheoreticalTest"));




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
                () -> -driverController.getRightX(),
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
        new Trigger(driverController::getCrossButton).onTrue(new InstantCommand(() -> intakeSubsystem.run(-0.25, -0.25)).andThen(new InstantCommand( () -> intakeSubsystem.run(0,0))));
//        new Trigger(driverController::getCrossButton).onFalse(new InstantCommand(() -> intakeSubsystem.run(-0, -0)));

        new Trigger(operatorController::getCircleButton).onTrue(new ShooterReset(shooterSubsystem));
        //new Trigger(operatorController::getCrossButton).toggleOnTrue(shooterCommand);
        new Trigger(operatorController::getSquareButton).toggleOnTrue(sourceIntake);
        new Trigger(operatorController::getTriangleButton).toggleOnTrue(ampCommand);
        new Trigger(operatorController::getL1Button).toggleOnTrue(new BetterIntakeCommand(intakeSubsystem, shooterSubsystem));
        new Trigger(operatorController::getR1Button).toggleOnTrue(new BetterIntakeReverse(intakeSubsystem, shooterSubsystem));
        new Trigger(operatorController::getL2Button).whileTrue(new ClimbDownCommand(climbSubsystem));
        new Trigger(operatorController::getR2Button).whileTrue(new ClimbUpCommand(climbSubsystem));

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
        NamedCommands.registerCommand("Run Intake", betterIntakeCommand);
        NamedCommands.registerCommand("Stop Intake", new InstantCommand(() -> {intakeSubsystem.run(0, 0); shooterSubsystem.setLaunchTalon(0);}));
        NamedCommands.registerCommand("Shoot Note", shooterCommand);
        NamedCommands.registerCommand("Auto Intake", new AutoIntakeCommand(intakeSubsystem, shooterSubsystem));
        NamedCommands.registerCommand("Shoot 25 Angle", new ShooterCommandToAngle(shooterSubsystem, 25));
        NamedCommands.registerCommand("Disable Vision", new InstantCommand(() -> photonvision.setDriverMode(true)));
        NamedCommands.registerCommand("Enable Vision", new InstantCommand(() -> photonvision.setDriverMode(false)));
    }

    public void setMotorBrake(boolean brake) {
        swerve.setMotorBrake(brake);
    }
}
