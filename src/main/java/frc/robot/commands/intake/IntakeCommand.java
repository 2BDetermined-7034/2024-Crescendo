package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends Command {

	private final IntakeSubsystem intake;

	public IntakeCommand(IntakeSubsystem intake) {
		this.intake = intake;
		addRequirements(intake);
	}

	@Override
	public void execute(){intake.run(Constants.Intake.upperIntakeSpeed, Constants.Intake.lowerIntakeSpeed);}

	@Override
	public void end(boolean interrupted){intake.run(0, 0);}
}
