package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ClimbUpCommand extends Command {
	protected ClimbSubsystem climb;

	public ClimbUpCommand(ClimbSubsystem climb) {
		this.climb = climb;
		addRequirements(climb);
	}

	@Override
	public void execute() {
//		climb.setPosition(Constants.Climb.hoistUpPosition);
		climb.setSpeed(-0.1);
	}

	@Override
	public void end(boolean interrupted) {
		climb.setSpeed(0.0);
	}
}
