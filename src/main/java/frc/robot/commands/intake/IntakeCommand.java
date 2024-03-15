package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.sensors.LaserCANSensor;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IntakeCommand extends Command {

	private final IntakeSubsystem intake;
	private final ShooterSubsystem shooter;
	private final LaserCANSensor intakeLaser, shooterLaser;

	public IntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter, LaserCANSensor intakeLaser, LaserCANSensor shooterLaser) {
		this.intake = intake;
		this.shooter = shooter;
		this.intakeLaser = intakeLaser; this.shooterLaser = shooterLaser;
		addRequirements(intake);
	}

	@Override
	public void execute() {
		//TODO uncomment once lasercan is online
		//Shooter should probably always be at zero at the end of a shooter command, its simpler
//		if (shooterLaser.getLatestDistance() > Constants.LaserConstants.detectionDistance
//				/*&& shooter.getAnglePositionDegrees() > Constants.Shooter.angleBackHardstop - 3*/) {
//			intake.run(Constants.Intake.upperIntakeSpeed, Constants.Intake.lowerIntakeSpeed);
//		}
//		if (intakeLaser.getLatestDistance() < Constants.LaserConstants.detectionDistance)
//			shooter.setNeoSpeeds(0.5);
//		else
//			shooter.setNeoSpeeds(0.0);
		intake.run(Constants.Intake.lowerIntakeSpeed, Constants.Intake.upperIntakeSpeed);
		shooter.setNeoSpeeds(.5);
	}

	@Override
	public void end(boolean interrupted) {
		intake.run(0, 0);
		shooter.setNeoSpeeds(0.0);
	}
}
