package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class ClimbToAmp extends Command {
	protected ClimbSubsystem climb;

	public ClimbToAmp(ClimbSubsystem climb) {
		this.climb = climb;
		addRequirements(climb);
	}

	@Override
	public void execute() {
		climb.setPosition(Constants.Climb.climbAmpPosition);
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
