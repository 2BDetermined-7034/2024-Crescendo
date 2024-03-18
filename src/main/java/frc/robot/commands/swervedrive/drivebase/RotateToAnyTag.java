package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;

public class RotateToAnyTag extends Command {
    public int tagID;
    PIDController controller =  new PIDController(3,0.2,0);
    private SwerveSubsystem swerve;
    private Rotation2d previousSetpoint;
    Pose3d tagPose;
    public RotateToAnyTag(SwerveSubsystem swerveSubsystem, int tagID) {
        this.swerve = swerveSubsystem;
        controller.setTolerance(0.01, 0.1);

        this.tagID = tagID;
        this.tagPose = Constants.aprilTagFieldLayout.getTagPose(tagID).get();
        previousSetpoint = Rotation2d.fromRadians(Math.atan2(tagPose.getY() - swerve.getPose().getY(), tagPose.getX() - swerve.getPose().getX())).rotateBy(Rotation2d.fromDegrees(180));
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {

        Rotation2d rotationSetpoint = Rotation2d.fromRadians(Math.atan2(tagPose.getY() - swerve.getPose().getY(), tagPose.getX() - swerve.getPose().getX())).rotateBy(Rotation2d.fromDegrees(180));
        if (previousSetpoint.getRadians() < 0) {
            if (rotationSetpoint.getRadians() > 0)
                rotationSetpoint.minus(Rotation2d.fromDegrees(360));
        } else {
            if (rotationSetpoint.getRadians() < 0)
                rotationSetpoint.plus(Rotation2d.fromDegrees(360));
        }

        previousSetpoint = rotationSetpoint;

        controller.setP(Math.PI - Math.abs(rotationSetpoint.minus(swerve.getPose().getRotation()).getRadians()));
        swerve.drive(new Translation2d(), controller.calculate(swerve.getHeading().getRadians(), rotationSetpoint.getRadians()) , true);

        SmartDashboard.putNumber("RotateToTag robot yaw", swerve.getHeading().getDegrees());
        SmartDashboard.putNumber("RotateToTag Controller Output", controller.calculate(swerve.getHeading().getRadians(), rotationSetpoint.getRadians()));
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint();
    }
}
