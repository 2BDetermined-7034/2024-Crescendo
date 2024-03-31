package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.sensors.LaserCANSensor;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;


public class AutoFactory {
	private final RobotContainer container;
	private final SwerveSubsystem swerve;
	private final ShooterSubsystem shooterSubsystem;
	private final LaserCANSensor intakeLaser;
	private final LaserCANSensor shooterLaser;
	private final IntakeSubsystem intakeSubsystem;

	public AutoFactory(RobotContainer container) {
		this.container = container;
		swerve = container.swerve;
		shooterSubsystem = container.shooterSubsystem;
		intakeLaser = container.intakeLaser;
		shooterLaser = container.shooterLaser;
		intakeSubsystem = container.intakeSubsystem;
	}

	public Command followPath(String pathName) {
		PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
		return AutoBuilder.followPath(path);
	}

	public Command followChoreoPath(String pathname, boolean resetOdometry) {
		PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathname);
		if (resetOdometry) {
			swerve.resetOdometry(path.getPreviewStartingHolonomicPose());
		}
		return AutoBuilder.followPath(path);
	}

	public Command getAutonomousCommand(String pathName) {
		// Create a path following command using AutoBuilder. This will also trigger event markers.
		return new PathPlannerAuto(pathName);
	}


	public Command constantShooter() {
		return Commands.startEnd(
				() -> shooterSubsystem.setLaunchTalon(80),
				() -> shooterSubsystem.setLaunchTalon(0),
				shooterSubsystem
		);
	}

	public Command stallIntake() {
		return new FunctionalCommand(
				() -> {
				},
				() -> {

					if (shooterLaser.getLatestMeasurement() < 20) {
						shooterSubsystem.setNeoSpeeds(0);
					} else {
						shooterSubsystem.setNeoSpeeds(0.05);
						intakeSubsystem.run(Constants.Intake.lowerIntakeSpeed, Constants.Intake.upperIntakeSpeed);
					}

				},
				(interrupted) -> {
					shooterSubsystem.setNeoSpeeds(0);
					intakeSubsystem.run(0, 0);
				},
				() -> shooterLaser.getLatestMeasurement() < 20
		);
	}

	public Command shootNote() {
		return new FunctionalCommand(
				() -> {
				},
				() -> {
					if (shooterLaser.getLatestMeasurement() < 50) {
						shooterSubsystem.setNeoSpeeds(0.1);
					} else {
						shooterSubsystem.setNeoSpeeds(0);
					}
				},
				(interrupted) -> {
					shooterSubsystem.setNeoSpeeds(0);
				},
				() -> false
		);
	}

	public Command angleShooter(double angle) {
		return Commands.startEnd(
				() -> shooterSubsystem.setAngleTalonPositionDegrees(angle),
				() -> shooterSubsystem.setAngleTalonPositionDegrees(0),
				shooterSubsystem
		);
	}

	public Command aw1() {
		return new ParallelCommandGroup(
				constantShooter(),
				new SequentialCommandGroup(
						new ParallelRaceGroup(
								new WaitCommand(1),
								shootNote()
						),
						new ParallelRaceGroup(
								followPath("Amp to W1"),
								stallIntake()
						),
						new ParallelRaceGroup(
								new WaitCommand(1),
								shootNote()
						)
				)
		);
	}

	public Command midw2() {

		return new ParallelCommandGroup(
				constantShooter(),
				new SequentialCommandGroup(
						new ParallelRaceGroup(
								new WaitCommand(1),
								shootNote()
						),
						new ParallelRaceGroup(
								followChoreoPath("MID to W2", true),
								stallIntake()
						),
						followChoreoPath("MID W2 to Community Line", false),
						new ParallelRaceGroup(
								new WaitCommand(1),
								shootNote()
						)
				)
		);
	}

	public Command sw3() {
		return new ParallelCommandGroup(
				constantShooter(),
				new SequentialCommandGroup(
						new ParallelRaceGroup(
								new WaitCommand(3),
								shootNote()
						),
						new ParallelRaceGroup(
								followPath("SOURCE to W3"),
								stallIntake()
						),
						new ParallelRaceGroup(
								new WaitCommand(1),
								shootNote()
						)
				)
		);
	}
}
