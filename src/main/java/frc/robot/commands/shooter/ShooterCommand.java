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
		SmartDashboard.putNumber("Set The Shooter Angle", 63);

		interpolation = new LinearInterpolation();

		interpolation.vertices = new Translation2d[7];
		interpolation.vertices[0] = new Translation2d(1.42, 45.0);
		interpolation.vertices[1] = new Translation2d(2.02, 35.0);
		interpolation.vertices[1] = new Translation2d(2.38, 32.0);
		interpolation.vertices[2] = new Translation2d(2.50, 30.0);
		interpolation.vertices[3] = new Translation2d(3.03, 26.4);
		interpolation.vertices[4] = new Translation2d(3.50, 24.0);
		interpolation.vertices[5] = new Translation2d(4.05, 21.5);
		interpolation.vertices[6] = new Translation2d(4.65, 21.0);

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
		int tagID = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 7 : 4;
		double distance = Constants.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d().minus(swerveSubsystem.getPose()).getTranslation().getNorm();
		SmartDashboard.putNumber("Shooter Distance", distance);
		shooter.setLaunchTalon(velocitySetpoint);
		//shooter.setAngleTalonPositionDegrees(0);

		//shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop);
		shooter.setAngleTalonPositionDegrees(interpolation.get(distance));
//		shooter.setAngleTalonPositionDegrees(SmartDashboard.getNumber("Set The Shooter Angle",  63));
		SmartDashboard.putNumber("Rational Inter Output", interpolation.get(distance));
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
