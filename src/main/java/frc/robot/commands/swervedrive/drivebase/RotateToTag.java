package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RotateToTag extends Command  {
	int tagID;
	Pose3d tagPose;
	PIDController controller =  new PIDController(0.4,0.0,0);
	private SwerveSubsystem swerve;
	private Rotation2d previousSetpoint;
	public RotateToTag(SwerveSubsystem swerveSubsystem) {
		this.swerve = swerveSubsystem;
		controller.setTolerance(0.01, 0.1);
		controller.setSetpoint(0.0);
		controller.enableContinuousInput(0.0, 2.0 * Math.PI);

		this.tagID = DriverStation.getAlliance().orElseThrow() == DriverStation.Alliance.Red ? 4 : 7;
		this.tagPose = Constants.aprilTagFieldLayout.getTagPose(tagID).get();

		addRequirements(swerveSubsystem);
	}

	@Override
	public void execute() {
		SmartDashboard.putNumber("Robot Yaw", swerve.getHeading().getDegrees());

		Rotation2d error = Rotation2d.fromRadians(Math.PI - Math.atan2(tagPose.getY() - swerve.getPose().getY(), tagPose.getX() - swerve.getPose().getX())).minus(swerve.getHeading());

		swerve.drive(new Translation2d(), controller.calculate(error.getRadians()), true);

		SmartDashboard.putNumber("RotateToTag error", error.getDegrees());
		SmartDashboard.putNumber("RotateToTag robot yaw", swerve.getHeading().getDegrees());
		SmartDashboard.putNumber("RotateToTag Controller Output", controller.calculate(error.getRadians()) * 180 / Math.PI);
	}

	@Override
	public boolean isFinished() {
		return controller.atSetpoint();
	}
}