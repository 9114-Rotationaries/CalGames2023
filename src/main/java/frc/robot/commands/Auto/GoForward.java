package frc.robot.commands.Auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class GoForward extends SequentialCommandGroup {
    public GoForward() {
        List<PathPlannerTrajectory> traj = PathPlanner.loadPathGroup("Go Forward", new PathConstraints(4, 3));

        addCommands(
            Commands.sequence(
                RobotContainer.builder.resetPose(traj.get(0)),
                RobotContainer.builder.followPathWithEvents(traj.get(0)),
                RobotContainer.builder.followPath(traj.get(1))
            )
        );
    }
}
