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
public class Co2Cu1M {
    private final List<PathPlannerTrajectory> m_traj;

    public Co2Cu1M(SwerveAutoBuilder builder) {
        m_traj = PathPlanner.loadPathGroup("2CO1CU-M", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);

        builder.fullAuto(m_traj);
    }
}
