package frc.robot.commands.Auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoFactory {

	public static Command followPath(String pathName) {
		PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
		return AutoBuilder.followPath(path);
	}



}
