package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class BetterIntakeReverse extends Command {

	private final IntakeSubsystem intake;
	private final ShooterSubsystem shooter;

	public BetterIntakeReverse(IntakeSubsystem intake, ShooterSubsystem shooter) {
		this.intake = intake;
		this.shooter = shooter;
		addRequirements(intake, shooter);
	}

	@Override
	public void execute(){
		intake.run(-Constants.Intake.upperIntakeSpeed, -Constants.Intake.lowerIntakeSpeed);
		shooter.setNeoSpeeds(-0.5);
	}

	@Override
	public void end(boolean interrupted){
		intake.run(0, 0);
		shooter.setNeoSpeeds(0);
	}
}
