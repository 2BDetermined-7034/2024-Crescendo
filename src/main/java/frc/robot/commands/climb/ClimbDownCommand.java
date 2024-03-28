package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
		climb.setPercent(-Constants.Climb.climbSpeed);
	}

	public boolean isFinished(){
		return climb.atCurrentLimit();
	}

	@Override
	public void end(boolean interrupted) {
		climb.setPercent(0.0);
	}
}
