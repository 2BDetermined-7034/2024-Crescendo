package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.RationalInterpolation;

public class ShooterCommand extends Command {
	protected ShooterSubsystem shooter;
	protected SwerveSubsystem swerveSubsystem;

	public ShooterCommand(ShooterSubsystem shooter, SwerveSubsystem swerveSubsystem) {
		this.shooter = shooter;
		this.swerveSubsystem = swerveSubsystem;
		addRequirements(shooter);
		SmartDashboard.putNumber("Set The Shooter Angle", 63);
	}

	@Override
	public void execute() {
		double velocitySetpoint = Constants.Shooter.shooterVelSetpoint;
		int tagID = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 7 : 4;
		double distance = Constants.aprilTagFieldLayout.getTags().get(tagID).pose.toPose2d().minus(swerveSubsystem.getPose()).getTranslation().getNorm();
		SmartDashboard.putNumber("Shooter Distance", distance);
		shooter.setLaunchTalon(velocitySetpoint);
		//shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop);
		shooter.setAngleTalonPositionDegrees(RationalInterpolation.getShooterAngle(distance));
//		shooter.setAngleTalonPositionDegrees(SmartDashboard.getNumber("Set The Shooter Angle",  63));
		SmartDashboard.putNumber("Rational Inter Output", RationalInterpolation.getShooterAngle(distance));
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
