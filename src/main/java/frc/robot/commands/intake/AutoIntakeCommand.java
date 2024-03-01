package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoIntakeCommand extends Command {

	private final IntakeSubsystem intake;
	private final ShooterSubsystem shooter;

	public AutoIntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
		this.intake = intake;
		this.shooter = shooter;
		addRequirements(intake, shooter);
	}

	@Override
	public void execute(){
		intake.run(0.3, Constants.Intake.lowerIntakeSpeed);
		shooter.setNeoSpeeds(0.3);
		shooter.runLaunchPercent(-0.1);
	}

	@Override
	public void end(boolean interrupted){
		intake.run(0, 0);
		shooter.setNeoSpeeds(0);
		shooter.runLaunchPercent(0);
	}
}
