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
		SmartDashboard.putNumber("Set The Shooter Angle", 30);
		SmartDashboard.putNumber("Set Shooter Velocity", 60);

		interpolation = new LinearInterpolation();

		final boolean isKrepe = false;
		if (isKrepe) {
			interpolation.vertices = new Translation2d[8];
			interpolation.vertices[0] = new Translation2d(1.42, 45.0);
			interpolation.vertices[1] = new Translation2d(2.02, 35.0);
			interpolation.vertices[2] = new Translation2d(2.38, 32.0);
			interpolation.vertices[3] = new Translation2d(2.50, 30.0);
			interpolation.vertices[4] = new Translation2d(3.03, 26.4);
			interpolation.vertices[5] = new Translation2d(3.50, 24.0);
			interpolation.vertices[6] = new Translation2d(4.05, 21.5);
			interpolation.vertices[7] = new Translation2d(4.65, 21.0);
		} else {
//			interpolation.vertices = new Translation2d[7];
//			interpolation.vertices[0] = new Translation2d(1.262461460840503, 55);
//			interpolation.vertices[1] = new Translation2d(1.875073740967453, 46);
//			interpolation.vertices[2] = new Translation2d(2.490053775301344, 36.0);
//			interpolation.vertices[3] = new Translation2d(3.040430474763867, 33.0);
//			interpolation.vertices[4] = new Translation2d(3.487449626171466, 30.5);
//			interpolation.vertices[5] = new Translation2d(4.030337165233073, 29.5);
//			interpolation.vertices[6] = new Translation2d(4.419117932614939, 28.25);

			interpolation.vertices = new Translation2d[7];
			interpolation.vertices[0] = new Translation2d(1.265, 50);
			interpolation.vertices[1] = new Translation2d(1.512, 45);
			interpolation.vertices[2] = new Translation2d(1.996, 38);
			interpolation.vertices[3] = new Translation2d(2.496, 34);
			interpolation.vertices[4] = new Translation2d(3.0, 32);
			interpolation.vertices[5] = new Translation2d(3.51, 26.5);
			interpolation.vertices[6] = new Translation2d(4.0, 25);
		}

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
//		double velocitySetpoint = SmartDashboard.getNumber("Set Shooter Velocity", 60.0);
		int tagID = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 7 : 4;
		double distance = Constants.aprilTagFieldLayout.getTags().get(tagID - 1).pose.toPose2d().minus(swerveSubsystem.getPose()).getTranslation().getNorm();
		SmartDashboard.putNumber("Shooter Distance", distance);
		shooter.setLaunchTalon(velocitySetpoint);
		double degreeOutput = interpolation.get(distance);
//		double degreeOutput = SmartDashboard.getNumber("Set The Shooter Angle",  30);
//		shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop);
		shooter.setAngleTalonPositionDegrees(degreeOutput);
//		SmartDashboard.putNumber("Rational Inter Output", interpolation.get(distance));
		if(shooter.getLaunchMotorVelocity() > velocitySetpoint - Constants.Shooter.shooterVelTolerance && Math.abs(shooter.getAnglePositionDegrees() - degreeOutput) < 2 && Math.abs(shooter.getAngleAcceleration()) < 3) {
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
