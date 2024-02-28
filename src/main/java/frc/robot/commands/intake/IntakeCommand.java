package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
		if (!(!Constants.Shooter.isLaserIntakeTriggered && Constants.Shooter.isLaserShooterTriggered)) {
			if (Constants.Shooter.isShooterAtHome) {
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
