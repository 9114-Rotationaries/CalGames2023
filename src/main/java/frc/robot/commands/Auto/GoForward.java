package frc.robot.commands.Auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotContainer;
import frc.robot.Constants.AutoConstants;

public class GoForward extends CommandBase {
    private final List<PathPlannerTrajectory> m_traj;

    public GoForward(SwerveAutoBuilder builder) {

        PathPlannerTrajectory traj1 = PathPlanner.generatePath(
            new PathConstraints(4, 3), 
            new PathPoint(new Translation2d(1.0, 1.0), Rotation2d.fromDegrees(0)), // position, heading
            new PathPoint(new Translation2d(3.0, 3.0), Rotation2d.fromDegrees(45)) // position, heading
        );

        m_traj = PathPlanner.loadPathGroup("Go Forward", AutoConstants.maxVelocity, AutoConstants.maxAcceleration);

        builder.fullAuto(traj1);
        SmartDashboard.putNumber("Auto", 1);
        /*Commands.sequence(
            builder.followPathWithEvents(m_traj.get(0)),
            new PrintCommand("First Marker"),
            Commands.runOnce(() -> m_container.getIntake().cubeOuttake(0.5))
        );*/

    }
}
