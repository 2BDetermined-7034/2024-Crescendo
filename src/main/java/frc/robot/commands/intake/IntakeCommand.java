package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.sensors.LaserCANSensor;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IntakeCommand extends Command {

	private final IntakeSubsystem intake;
	private final LaserCANSensor intakeLaser;
	private final LaserCANSensor shooterLaser;
	private final ShooterSubsystem shooter;

	public IntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter, LaserCANSensor intakeLaser, LaserCANSensor shooterLaser) {
		this.intake = intake;
		this.shooter = shooter;
		this.intakeLaser = intakeLaser;
		this.shooterLaser = shooterLaser;
		addRequirements(intake);
	}

	@Override
	public void execute() {
		if (shooterLaser.getLatestMeasurement() > 50)
			intake.run(Constants.Intake.lowerIntakeSpeed, Constants.Intake.upperIntakeSpeed);
		else
			intake.run(0.0, 0.0);

//		if (intakeLaser.getLatestMeasurement() < 50)
//			shooter.setNeoSpeeds(0.05);
//		else
//			shooter.setNeoSpeeds(0.0);
		shooter.setNeoSpeeds(0.2);
	}

	@Override
	public boolean isFinished() {
		if (shooterLaser.getLatestMeasurement() < 50) {
			intake.run(0, 0);
			shooter.setNeoSpeeds(0.0);
			return true;
		} else {
			return false;
		}
	}
	@Override
	public void end(boolean interrupted) {
		intake.run(0, 0);
		shooter.setNeoSpeeds(0.0);
	}
}
