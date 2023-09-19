package frc.robot.commands.Auto;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

public class GoForward extends CommandBase {
    private final List<PathPlannerTrajectory> m_traj;

    public GoForward(SwerveAutoBuilder builder, RobotContainer m_container) {
        m_traj = PathPlanner.loadPathGroup("Go Forward", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);

        builder.fullAuto(m_traj);
        /*Commands.sequence(
            builder.followPathWithEvents(m_traj.get(0)),
            new PrintCommand("First Marker"),
            Commands.runOnce(() -> m_container.getIntake().cubeOuttake(0.5))
        );*/

    }
}
