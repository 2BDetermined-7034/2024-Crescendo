package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShooterAmpCommand extends Command {
	protected ShooterSubsystem shooter;

	public ShooterAmpCommand(ShooterSubsystem shooter) {
		this.shooter = shooter;
		addRequirements(shooter);
	}

	@Override
	public void execute() {
		double velocitySetpoint = 40;
		double angleSetpoint = 53;
		shooter.setLaunchTalon(velocitySetpoint);
		shooter.setAngleTalonPositionDegrees(angleSetpoint);
		//if(Math.abs(shooter.getLaunchMotorVelocity() - velocitySetpoint) < 5) {
		if(shooter.withinShootingTolerances(angleSetpoint, velocitySetpoint)){
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
