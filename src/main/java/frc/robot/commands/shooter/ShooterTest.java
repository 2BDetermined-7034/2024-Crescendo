package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterTest extends Command {
	protected ShooterSubsystem shooter;

	public ShooterTest(ShooterSubsystem shooter) {
		this.shooter = shooter;
		addRequirements(shooter);
	}

	@Override
	public void execute() {
//		double velocitySetpoint = 50;
//		shooter.setLaunchTalon(velocitySetpoint);
//		shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop);
//		if(shooter.getLaunchMotorVelocity() > velocitySetpoint - 2) {
//			shooter.setNeoSpeeds(0.5);
//		}
		shooter.setAngleTalonPositionDegrees(63);
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setAngleTalonPositionDegrees(63);
		shooter.setLaunchTalon(0);
		shooter.setNeoSpeeds(0.0);
	}
}
