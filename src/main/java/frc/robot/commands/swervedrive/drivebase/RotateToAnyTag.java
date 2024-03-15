package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants;

public class RotateToAnyTag extends Command {
    PIDController controller =  new PIDController(3,0.2,0);
    private SwerveSubsystem swerve;
    public RotateToAnyTag(SwerveSubsystem swerveSubsystem) {
        this.swerve = swerveSubsystem;
        controller.setTolerance(0.01, 0.1);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        int tagID = -1;
        if (RobotContainer.photonvision.hasTargets()){
            tagID = RobotContainer.photonvision.getBestTarget().getFiducialId();
            Rotation2d desiredRotation = Rotation2d.fromRadians(Math.atan2(Constants.aprilTagFieldLayout.getTagPose(tagID).get().getY() - swerve.getPose().getY(), Constants.aprilTagFieldLayout.getTagPose(tagID).get().getX() + Math.toRadians(180)) - swerve.getPose().getX());
            swerve.drive(new Translation2d(), controller.calculate(swerve.getHeading().getRadians(), desiredRotation.getRadians()) , true);
        }
    }

    @Override
    public boolean isFinished() {return controller.atSetpoint();}
}
