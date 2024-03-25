package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;

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


}
