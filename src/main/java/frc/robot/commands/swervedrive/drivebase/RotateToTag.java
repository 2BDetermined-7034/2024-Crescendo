package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RotateToTag extends Command  {
	PIDController controller =  new PIDController(3,0.2,0);
	private SwerveSubsystem swerve;
	public RotateToTag(SwerveSubsystem swerveSubsystem) {
		this.swerve = swerveSubsystem;
		//controller.enableContinuousInput(0,2 * Math.PI);
		controller.setTolerance(0.01, 0.1);
		addRequirements(swerveSubsystem);
	}

	@Override
	public void execute() {
//		if(DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)){
//			swerve.drive(new Translation2d(0,0),
//					Rotation2d.fromRadians(controller.calculate(swerve.getYaw().getRadians(),
//							Constants.AprilTags.layout.get(4).pose.getRotation().getZ())).getRadians(),
//					true);
//		} else {
//			swerve.drive(new Translation2d(0,0),
//					Rotation2d.fromRadians(controller.calculate(swerve.getYaw().getRadians(),
//							Constants.AprilTags.layout.get(7).pose.getRotation().getZ())).getRadians(),
//					true);
//		}


		int tagID = DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Red ? 4 : 7;
		Pose3d tagPose = Constants.aprilTagFieldLayout.getTagPose(tagID).get();
		Rotation2d rotationSetpoint = Rotation2d.fromRadians(Math.atan2(tagPose.getY() - swerve.getPose().getY(), tagPose.getX() - swerve.getPose().getX())).rotateBy(Rotation2d.fromDegrees(180));
		//double rotation  = Constants.AprilTags.layout.get(tagID).pose.getRotation().toRotation2d().getRadians() - swerve.getHeading().getRadians();
		swerve.drive(new Translation2d(), controller.calculate(swerve.getHeading().getRadians(), rotationSetpoint.getRadians()) , true);

		SmartDashboard.putNumber("RotateToTag robot yaw", swerve.getHeading().getDegrees());
		SmartDashboard.putNumber("RotateToTag Controller Output", controller.calculate(swerve.getHeading().getRadians(), rotationSetpoint.getRadians()));
	}

	@Override
	public boolean isFinished() {
		return controller.atSetpoint();
	}
}