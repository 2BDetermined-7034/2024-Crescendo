package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
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
			Pose2d startingPose = path.getPreviewStartingHolonomicPose();
			if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
				startingPose = GeometryUtil.flipFieldPose(startingPose);
			}
			Rotation3d rotation3d = new Rotation3d(0, 0, startingPose.getRotation().getRadians());
			swerve.setGyro(rotation3d);
			swerve.resetOdometry(startingPose);
		}
		return AutoBuilder.followPath(path);
	}

	public Command getAutonomousCommand(String pathName) {
		// Create a path following command using AutoBuilder. This will also trigger event markers.
		return new PathPlannerAuto(pathName);
	}


	public Command constantShooter() {
		return Commands.startEnd(
				() -> shooterSubsystem.setLaunchTalon(Constants.Shooter.shooterVelSetpoint),
				() -> shooterSubsystem.setLaunchTalon(0)
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
						shooterSubsystem.setNeoSpeeds(0.08);
						intakeSubsystem.run(Constants.Intake.lowerIntakeSpeed + 1.5, Constants.Intake.upperIntakeSpeed + 1.5);
					}

				},
				(interrupted) -> {
					shooterSubsystem.setNeoSpeeds(0);
					intakeSubsystem.run(0, 0);
				},
				() -> false
		);
	}

	public Command shootNote() {
		return new FunctionalCommand(
				() -> {
				},
				() -> {
					if (Math.abs(shooterSubsystem.getLaunchMotorVelocity() - Constants.Shooter.shooterVelSetpoint) < 4) {
						shooterSubsystem.setNeoSpeeds(0.4);
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
		return new FunctionalCommand(
				() -> {},
				() -> shooterSubsystem.setAngleTalonPositionDegrees(angle),
				(interrupted) -> {},
				() -> shooterSubsystem.withinShootingTolerances(angle)
		);
	}

//	public Command aw1() {
//    Command autoCommand = new SequentialCommandGroup(
//        new WaitCommand(1),
//        new ParallelRaceGroup(
//            new WaitCommand(1),
//            shootNote()
//        ),
//        new ParallelRaceGroup(
//            followChoreoPath("Amp to W1", true),
//            stallIntake()
//        ),
//        angleShooter(36),
//        new ParallelRaceGroup(
//            new WaitCommand(1),
//            shootNote()
//        ),
//        angleShooter(Constants.Shooter.angleBackHardstop)
//    );
//
//    return new PathPlannerAuto("Amp to W1") {
//        @Override
//        public void initialize() {
//            super.initialize();
//            autoCommand.initialize();
//        }
//
//        @Override
//        public void execute() {
//            super.execute();
//            autoCommand.execute();
//        }
//
//        @Override
//        public void end(boolean interrupted) {
//            super.end(interrupted);
//            autoCommand.end(interrupted);
//        }
//
//        @Override
//        public boolean isFinished() {
//            return autoCommand.isFinished();
//        }
//    };
//}

//	public Command aw1() {
//		return new ParallelCommandGroup(
//				constantShooter(),
//				new SequentialCommandGroup(
//						new WaitCommand(1),
//						new ParallelRaceGroup(
//								new WaitCommand(1),
//								shootNote()
//						),
//						new ParallelRaceGroup(
//								followChoreoPath("Amp to W1", true),
//								stallIntake()
//						),
//						angleShooter(36),
//						new ParallelRaceGroup(
//								new WaitCommand(1),
//								shootNote()
//						),
//						angleShooter(Constants.Shooter.angleBackHardstop)
//				)
//		);
//	}
//
//	/*
//	Mid 2 works
//	 */
//	public Command midw2() {
//		return new ParallelCommandGroup(
//				constantShooter(),
//				new SequentialCommandGroup(
//						new WaitCommand(1),
//						new ParallelRaceGroup(
//								new WaitCommand(1),
//								shootNote()
//						),
//						new ParallelRaceGroup(
//								followChoreoPath("MID to W2", true),
//								stallIntake()
//						),
//						new ParallelRaceGroup(
//								followChoreoPath("MID W2 to Community Line", false),
//								stallIntake()
//						),
//						angleShooter(36),
//						new ParallelRaceGroup(
//								new WaitCommand(1),
//								shootNote()
//						),
//						angleShooter(Constants.Shooter.angleBackHardstop)
//				)
//		);
//	}
//
//	public Command sw3() {
//		return new ParallelCommandGroup(
//				constantShooter(),
//				new SequentialCommandGroup(
//						new ParallelRaceGroup(
//								new WaitCommand(3),
//								shootNote()
//						),
//						new ParallelRaceGroup(
//								followPath("SOURCE to W3"),
//								stallIntake()
//						),
//						new ParallelRaceGroup(
//								new WaitCommand(1),
//								shootNote()
//						)
//				)
//		);
//	}
}
