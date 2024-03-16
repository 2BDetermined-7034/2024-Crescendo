package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LinearInterpolation;

public class AmpShot extends Command {
	protected ShooterSubsystem shooter;
	protected SwerveSubsystem swerveSubsystem;
//	protected LagrangeInterpolation interpolation;
	protected LinearInterpolation interpolation;

	public AmpShot(ShooterSubsystem shooter, SwerveSubsystem swerveSubsystem) {
		this.shooter = shooter;
		this.swerveSubsystem = swerveSubsystem;
		addRequirements(shooter);
		SmartDashboard.putNumber("Set The Shooter Angle", 63);

//		interpolation = new LinearInterpolation();
//
//		interpolation.vertices = new Translation2d[7];
//		interpolation.vertices[0] = new Translation2d(.257463908733832, 45.0);
//		interpolation.vertices[1] = new Translation2d(1.9039310238475524, 35.0);
//		interpolation.vertices[2] = new Translation2d(2.4671186773596276, 31.0);
//		interpolation.vertices[3] = new Translation2d(2.954671604288543, 28.0);
//		interpolation.vertices[4] = new Translation2d(3.5234219066572603, 25.0);
//		interpolation.vertices[5] = new Translation2d(4.019970514449032, 22.50);
//		interpolation.vertices[6] = new Translation2d(4.613365440225928, 20.750);

//		interpolation.function = (Double x) -> {
//			return 1.0 / x;
//		};
//
//		interpolation.inverse = (Double x) -> {
//			return 1.0 / x;
//		};
	}

	@Override
	public void execute() {
		double velocitySetpoint = Constants.Shooter.shooterAmpVelSetpoint;

		shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop);
		shooter.setLaunchTalon(velocitySetpoint);
//		shooter.setAngleTalonPositionDegrees(SmartDashboard.getNumber("Set The Shooter Angle",  63));
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