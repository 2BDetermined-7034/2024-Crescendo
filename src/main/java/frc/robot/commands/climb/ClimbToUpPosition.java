package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ClimbToUpPosition extends Command {
	protected ClimbSubsystem climb;

	public ClimbToUpPosition(ClimbSubsystem climb) {
		this.climb = climb;
		addRequirements(climb);
	}

	@Override
	public void execute() {
		climb.setPosition(0.0);
	}

	@Override
	public boolean isFinished(){
		return climb.atCurrentLimit();
	}

	@Override
	public void end(boolean interrupted) {
		if(climb.atCurrentLimit()){
			climb.coast();
		} else {
			climb.setPosition(climb.getPosition());
		}
	}
}
