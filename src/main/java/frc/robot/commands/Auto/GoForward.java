package frc.robot.commands.Auto;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.Constants.AutoConstants;

public class GoForward {
    private final List<PathPlannerTrajectory> m_traj;

    public GoForward(SwerveAutoBuilder builder) {
        m_traj = PathPlanner.loadPathGroup("Go Forward", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);

        builder.fullAuto(m_traj);
    }
}
