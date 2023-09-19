package frc.robot.commands.Auto;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;

/*names: Cu --> Cube
         Co --> Cone
         B  --> Back (starting position) --> will be changed later
         M  --> Middle
         F  --> Front
*/
public class Co2M extends CommandBase {
    private final List<PathPlannerTrajectory> m_traj;

    public Co2M(SwerveAutoBuilder builder) {
        m_traj = PathPlanner.loadPathGroup("2CO-M", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);

        builder.fullAuto(m_traj);
    }
}
