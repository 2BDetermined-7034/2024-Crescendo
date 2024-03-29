package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.sensors.LaserCANSensor;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoFactory {

	public static Command followPath(String pathName) {
		PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
		return AutoBuilder.followPath(path);
	}

	public static Command getAutonomousCommand(String pathName)
	{
		// Create a path following command using AutoBuilder. This will also trigger event markers.
		return new PathPlannerAuto(pathName);
	}


	public static Command constantShooter(ShooterSubsystem shooter) {
		return Commands.startEnd(
				() -> shooter.setLaunchTalon(80),
				() -> shooter.setLaunchTalon(0),
				shooter
		);
	}

	public static Command stallIntake(IntakeSubsystem intake, ShooterSubsystem shooter, LaserCANSensor intakeLaser, LaserCANSensor shooterLaser) {
		return new FunctionalCommand(
				() -> {},
				() -> {

					if(shooterLaser.getLatestMeasurement() < 50) {
						shooter.setNeoSpeeds(0);
					} else {
						shooter.setNeoSpeeds(0.05);
						intake.run(Constants.Intake.lowerIntakeSpeed, Constants.Intake.upperIntakeSpeed);
					}

				},
				(interrupted) -> {
					shooter.setNeoSpeeds(0);
					intake.run(0,0);
				},
				() -> false
		);
	}

	public static Command shootNote(ShooterSubsystem shooter, LaserCANSensor shooterLaser) {
		return new FunctionalCommand(
				() -> {},
				() -> {
					if(shooterLaser.getLatestMeasurement() < 50) {
						shooter.setNeoSpeeds(0.05);
					} else {
						shooter.setNeoSpeeds(0);
					}
				},
				(interrupted) -> {
					shooter.setNeoSpeeds(0);
				},
				() -> false
		);
	}

	public static Command angleShooter(ShooterSubsystem shooter, double angle) {
		return Commands.startEnd(
				() -> shooter.setAngleTalonPositionDegrees(angle),
				() -> shooter.setAngleTalonPositionDegrees(0),
				shooter
		);
	}

	public class Autos {
		public class TwoPiece {
			public static Command ampTwoPiece() {
				return new ParallelCommandGroup(
						getSpinUpShooterCommand(),
						new SequentialCommandGroup(
								new ParallelRaceGroup(
										new WaitCommand(1),
										getShootNoteCommand()
								),
								new ParallelRaceGroup(
										followPath("Amp to W1"),
										getStallIntakeCommand()
								),
								new ParallelRaceGroup(
										new WaitCommand(1),
										getShootNoteCommand()
								)
						)
				);
			}

			public static Command midTwoPiece() {
				return new ParallelCommandGroup(
						getSpinUpShooterCommand(),
						new SequentialCommandGroup(
								new ParallelRaceGroup(
										new WaitCommand(1),
										getShootNoteCommand()
								),
								new ParallelRaceGroup(
										followPath("Mid to W2"),
										getStallIntakeCommand()
								),
								followPath("MID W2 to Community Line"),
								new ParallelRaceGroup(
										new WaitCommand(1),
										getShootNoteCommand()
								)
						)
				);
			}

			public static Command sourceTwoPiece() {
				return new ParallelCommandGroup(
						getSpinUpShooterCommand(),
						new SequentialCommandGroup(
								new ParallelRaceGroup(
										new WaitCommand(1),
										getShootNoteCommand()
								),
								new ParallelRaceGroup(
										followPath("SOURCE to W3"),
										getStallIntakeCommand()
								),
								new ParallelRaceGroup(
										new WaitCommand(1),
										getShootNoteCommand()
								)
						)
				);
			}
		}

		public class ThreePiece {

		}
	}

	private static Command getSpinUpShooterCommand() {
		return constantShooter(RobotContainer.shooterSubsystem);
	}

	private static Command getShootNoteCommand() {
		return shootNote(RobotContainer.shooterSubsystem, RobotContainer.shooterLaser);
	}

	private static Command getStallIntakeCommand() {
		return stallIntake(RobotContainer.intakeSubsystem, RobotContainer.shooterSubsystem, RobotContainer.intakeLaser, RobotContainer.shooterLaser);
	}

}
