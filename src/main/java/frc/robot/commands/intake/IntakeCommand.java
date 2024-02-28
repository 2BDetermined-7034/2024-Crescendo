package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class IntakeCommand extends Command {

	protected IntakeSubsystem intake;
	protected ShooterSubsystem shooter;

	public IntakeCommand(IntakeSubsystem intake, ShooterSubsystem shooter) {
		this.intake = intake;
		this.shooter = shooter;
		addRequirements(this.intake);
		addRequirements(this.shooter);
	}

	@Override
	public void execute() {
		intake.run(Constants.Intake.upperIntakeSpeed, Constants.Intake.lowerIntakeSpeed);
		if (!(!RobotContainer.isLaserIntakeTriggered && RobotContainer.isLaserShooterTriggered)) {
			if (RobotContainer.isShooterAtHome) {
				shooter.setNeoSpeeds(0.5);
			} else {
				shooter.setAngleTalonPositionDegrees(Constants.Shooter.angleBackHardstop);
			}
		}
	}

	@Override
	public void end(boolean interrupted) {
		intake.run(0, 0);
		shooter.setNeoSpeeds(0.0);
	}
}
