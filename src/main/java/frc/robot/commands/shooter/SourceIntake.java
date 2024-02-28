package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SourceIntake extends Command {
	protected ShooterSubsystem shooter;

	public SourceIntake(ShooterSubsystem shooter) {
		this.shooter = shooter;
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		double velocitySetpoint = -30;
		shooter.setLaunchTalon(velocitySetpoint);
		//shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop);
//		if(shooter.getLaunchMotorVelocity() > velocitySetpoint - 2) {
//			shooter.setNeoSpeeds(0.5);
//		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setLaunchTalon(0);
		//shooter.setNeoSpeeds(0.0);
	}
}
