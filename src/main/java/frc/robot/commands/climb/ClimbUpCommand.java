package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
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
		climb.setOverrideVelocity(-0.4);
	}

	public boolean isFinished(){
		return climb.atCurrentLimit();
	}

	@Override
	public void end(boolean interrupted) {
		climb.setOverrideVelocity(0.0);
	}
}
