package frc.robot.commands.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.vision.Photonvision;
import frc.robot.utils.LinearInterpolation;

public class ShooterCommand extends Command {
	protected ShooterSubsystem shooter;
	protected SwerveSubsystem swerveSubsystem;
//	protected LagrangeInterpolation interpolation;
	protected LinearInterpolation forwardInterpolation;
	protected LinearInterpolation diagInterpolation;
	protected LinearInterpolation angleInterpolation;
	Photonvision photonvision;

	public ShooterCommand(ShooterSubsystem shooter, SwerveSubsystem swerveSubsystem, Photonvision photon) {
		this.shooter = shooter;
		this.swerveSubsystem = swerveSubsystem;
		this.photonvision = photon;
		addRequirements(shooter);

		forwardInterpolation = new LinearInterpolation();
		diagInterpolation = new LinearInterpolation();
		angleInterpolation = new LinearInterpolation();

		angleInterpolation.vertices = new Translation2d[3];

		final boolean isKrepe = true;
		if (isKrepe) {
			forwardInterpolation.vertices = new Translation2d[8];
			forwardInterpolation.vertices[0] = new Translation2d(1.1772372204566184, 53.0);
			forwardInterpolation.vertices[1] = new Translation2d(1.8243014487070923, 42);
			forwardInterpolation.vertices[2] = new Translation2d(2.3626154576138605, 35.0);
			forwardInterpolation.vertices[3] = new Translation2d(2.923072900415936, 32);
			forwardInterpolation.vertices[4] = new Translation2d(3.03, 26.4);
			forwardInterpolation.vertices[5] = new Translation2d(3.50, 24.0);
			forwardInterpolation.vertices[6] = new Translation2d(4.05, 21.5);
			forwardInterpolation.vertices[7] = new Translation2d(4.65, 21.0);

			diagInterpolation.vertices = new Translation2d[8];
			diagInterpolation.vertices[0] = new Translation2d(1.42, 45.0);
			diagInterpolation.vertices[1] = new Translation2d(2.02, 35.0);
			diagInterpolation.vertices[2] = new Translation2d(2.38, 32.0);
			diagInterpolation.vertices[3] = new Translation2d(2.50, 30.0);
			diagInterpolation.vertices[4] = new Translation2d(3.03, 26.4);
			diagInterpolation.vertices[5] = new Translation2d(3.50, 24.0);
			diagInterpolation.vertices[6] = new Translation2d(4.05, 21.5);
			diagInterpolation.vertices[7] = new Translation2d(4.65, 21.0);
		} else {
			forwardInterpolation.vertices = new Translation2d[7];
			forwardInterpolation.vertices[0] = new Translation2d(1.257463908733832, 63.0);
			forwardInterpolation.vertices[1] = new Translation2d(1.9039310238475524, 45.0);
			forwardInterpolation.vertices[2] = new Translation2d(2.4671186773596276, 31.0);
			forwardInterpolation.vertices[3] = new Translation2d(2.954671604288543, 28.0);
			forwardInterpolation.vertices[4] = new Translation2d(3.5234219066572603, 25.0);
			forwardInterpolation.vertices[5] = new Translation2d(4.019970514449032, 22.50);
			forwardInterpolation.vertices[6] = new Translation2d(4.613365440225928, 20.750);
		}

//		interpolation.function = (Double x) -> {
//			return 1.0 / x;
//		};
//
//		interpolation.inverse = (Double x) -> {
//			return 1.0 / x;
//		};
		SmartDashboard.putNumber("Set The Shooter Angle", 63);
	}

	@Override
	public void execute() {
		double velocitySetpoint = Constants.Shooter.shooterVelSetpoint;
		int tagID = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 7 : 4;
		Pose2d tagPose;
		double distance = 0.9144;
		if(photonvision.getCamera().isConnected()){
			tagPose = Constants.aprilTagFieldLayout.getTagPose(tagID).get().toPose2d();
			distance = tagPose.minus(swerveSubsystem.getPose()).getTranslation().getNorm();
		} else {
			tagPose = new Pose2d(0, 0, new Rotation2d(0));
		}

		SmartDashboard.putNumber("Shooter Distance", distance);
		shooter.setLaunchTalon(velocitySetpoint);
//		shooter.setAngleTalonPositionDegrees(0);

		//shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop);
		Pose2d robotPose = swerveSubsystem.getPose();

//		angleInterpolation.vertices[0] = new Translation2d(Math.PI * -0.5, diagInterpolation.get(distance));
//		angleInterpolation.vertices[1] = new Translation2d(0.0, forwardInterpolation.get(distance));
//		angleInterpolation.vertices[2] = new Translation2d(Math.PI * 0.5, diagInterpolation.get(distance));
//		shooter.setAngleTalonPositionDegrees(angleInterpolation.get(Math.atan2(robotPose.getY() - tagPose.getY(), robotPose.getX() - tagPose.getX())));


//		shooter.setAngleTalonPositionDegrees(forwardInterpolation.get(distance));

		shooter.setAngleTalonPositionDegrees(SmartDashboard.getNumber("Set The Shooter Angle",  63));
		SmartDashboard.putNumber("Rational Inter Output", forwardInterpolation.get(distance));
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
