package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterReset extends Command {
	protected ShooterSubsystem shooter;

	public ShooterReset(ShooterSubsystem shooter) {
		this.shooter = shooter;
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop);
	}

	@Override
	public void end(boolean interrupted) {

	}
}
