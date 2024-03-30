package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.sensors.LaserCANSensor;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class BetterIntakeCommand extends Command {

	private final IntakeSubsystem intake;
	private final ShooterSubsystem shooter;
	private final LaserCANSensor shooterLaser;

	public BetterIntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter, LaserCANSensor shooterLaser) {
		this.intake = intake;
		this.shooter = shooter;
		this.shooterLaser = shooterLaser;
		addRequirements(intake, shooter);
	}

	@Override
	public void execute(){
		intake.run(Constants.Intake.upperIntakeSpeed, Constants.Intake.lowerIntakeSpeed);
		if(shooterLaser.getLatestMeasurement() > 50) {
			shooter.setNeoSpeeds(0);
			shooter.runLaunchPercent(-0.05);
		} else {
			shooter.setNeoSpeeds(-0.3);
			shooter.runLaunchPercent(-0.2);
		}
	}

	@Override
	public void end(boolean interrupted){
		intake.run(0, 0);
		shooter.setNeoSpeeds(0);
		shooter.runLaunchPercent(0);
	}
}
