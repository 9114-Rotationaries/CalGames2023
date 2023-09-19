package frc.robot.commands.Auto;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import frc.robot.Constants.AutoConstants;

/*names: Cu --> Cube
         Co --> Cone
         B  --> Back (starting position) --> will be changed later
         M  --> Middle
         F  --> Front
*/
public class Co1Cu1B {
    private final List<PathPlannerTrajectory> m_traj;

    public Co1Cu1B(SwerveAutoBuilder builder) {
        m_traj = PathPlanner.loadPathGroup("1CO1CU-B", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);

        builder.fullAuto(m_traj);
    }
}
