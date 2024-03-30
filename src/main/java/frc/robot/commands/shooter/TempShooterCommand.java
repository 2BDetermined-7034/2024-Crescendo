package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class TempShooterCommand extends Command {
	protected ShooterSubsystem shooter;
	protected double degrees;

	public TempShooterCommand(ShooterSubsystem shooter, double degrees) {
		this.shooter = shooter;
		this.degrees = degrees;
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		double velocitySetpoint = Constants.Shooter.shooterVelSetpoint;
//		shooter.setLaunchTalon(velocitySetpoint);
		shooter.setAngleTalonPositionDegrees(degrees);
		if(shooter.getLaunchMotorVelocity() > velocitySetpoint - Constants.Shooter.shooterVelTolerance) {
			shooter.setNeoSpeeds(0.5);
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setLaunchTalon(0);
		shooter.setNeoSpeeds(0.0);
		shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop);
	}
}
