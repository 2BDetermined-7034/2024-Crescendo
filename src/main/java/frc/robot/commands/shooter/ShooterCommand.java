package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LagrangeInterpolation;
import frc.robot.utils.LinearInterpolation;
import frc.robot.utils.RationalInterpolation;
import frc.robot.utils.ThieleInterpolation;

public class ShooterCommand extends Command {
	protected ShooterSubsystem shooter;
	protected SwerveSubsystem swerveSubsystem;
//	protected LagrangeInterpolation interpolation;
	protected LinearInterpolation interpolation;

	public ShooterCommand(ShooterSubsystem shooter, SwerveSubsystem swerveSubsystem) {
		this.shooter = shooter;
		this.swerveSubsystem = swerveSubsystem;
		addRequirements(shooter);
		SmartDashboard.putNumber("Set The Shooter Angle", Constants.Shooter.angleBackHardstop);
		SmartDashboard.putNumber("Set Shooter Velocity", 30);
		SmartDashboard.putBoolean("Is Shooting", false);

		interpolation = new LinearInterpolation();

			interpolation.vertices = new Translation2d[11];
			interpolation.vertices[0] = new Translation2d(1.365, 55.0);
			interpolation.vertices[1] = new Translation2d(1.75, 45.0);
			interpolation.vertices[2] = new Translation2d(2.09, 38.0);
			interpolation.vertices[3] = new Translation2d(2.46, 32.0);
			interpolation.vertices[4] = new Translation2d(2.81, 29.0);
			interpolation.vertices[5] = new Translation2d(3.17, 27.0);
			interpolation.vertices[6] = new Translation2d(3.5, 24.3);
			interpolation.vertices[7] = new Translation2d(3.82, 23.0);
			interpolation.vertices[8] = new Translation2d(4.17, 22.0);
			interpolation.vertices[9] = new Translation2d(4.52, 21.0);
			interpolation.vertices[10] = new Translation2d(4.81, 20.25);




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
		double velocitySetpoint = Constants.Shooter.shooterVelSetpoint;
//		double velocitySetpoint = SmartDashboard.getNumber("Set Shooter Velocity", 25);
		double angleSetpoint = SmartDashboard.getNumber("Set The Shooter Angle", 54.58);
		int tagID = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 7 : 4;
		double distance = Constants.aprilTagFieldLayout.getTags().get(tagID - 1).pose.toPose2d().minus(swerveSubsystem.getPose()).getTranslation().getNorm();
		SmartDashboard.putNumber("Shooter Distance", distance);
		//double angleSetpoint = interpolation.get(distance);

		SmartDashboard.putNumber("Rational Inter Output", interpolation.get(distance));

		shooter.setLaunchTalon(velocitySetpoint);
		shooter.setAngleTalonPositionDegrees(angleSetpoint);

		if(shooter.withinShootingTolerances(angleSetpoint, velocitySetpoint)) {
			shooter.setNeoSpeeds(0.5);
			SmartDashboard.putBoolean("Is Shooting", true);
		} else {
			SmartDashboard.putBoolean("Is Shooting", false);
		}
	}

	@Override
	public void end(boolean interrupted) {
		shooter.setLaunchTalon(0);
		shooter.setNeoSpeeds(0.0);
		shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop);
		SmartDashboard.putBoolean("Is Shooting", false);
	}
}
