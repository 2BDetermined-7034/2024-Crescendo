package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class GroundTrapShot extends AmpShot {
	public GroundTrapShot(ShooterSubsystem shooter, SwerveSubsystem swerveSubsystem) {
		super(shooter, swerveSubsystem);
	}

	@Override
	public void execute() {
		double velocitySetpoint = Constants.Shooter.shooterTrapVelSetpoint;

		shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop - 10);
		shooter.setLaunchTalon(velocitySetpoint);
//		shooter.setAngleTalonPositionDegrees(SmartDashboard.getNumber("Set The Shooter Angle",  63));
		if(shooter.getLaunchMotorVelocity() > velocitySetpoint - Constants.Shooter.shooterVelTolerance) {
			shooter.setNeoSpeeds(0.5);
		}
	}
}
