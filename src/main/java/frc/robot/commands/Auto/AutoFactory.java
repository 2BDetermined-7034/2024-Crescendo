package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class AutoFactory {

	public static Command followPath(String pathName) {
		PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
		return AutoBuilder.followPath(path);
	}

	public static Command getAutonomousCommand(String pathName)
	{
		// Create a path following command using AutoBuilder. This will also trigger event markers.
		return new PathPlannerAuto(pathName);
	}

	public static Command spinShooterWheels(ShooterSubsystem shooterSubsystem) {

		return Commands.startEnd(
				() -> {shooterSubsystem.setLaunchTalon(80);},
				() -> {shooterSubsystem.setLaunchTalon(0);}
		);
	}

}
