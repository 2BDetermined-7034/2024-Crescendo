package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ClimbDownCommand extends Command {
	protected ClimbSubsystem climb;

	public ClimbDownCommand(ClimbSubsystem climb) {

		this.climb = climb;
		addRequirements(climb);


	}

	@Override
	public void execute() {
//		climb.setPosition(Constants.Climb.hoistDownPosition);
		climb.setOverrideVelocity(0.1);
	}

	@Override
	public void end(boolean interrupted) {
		climb.setOverrideVelocity(0.0);
	}
}
