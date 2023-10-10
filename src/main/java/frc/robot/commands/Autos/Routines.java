package frc.robot.commands.Autos;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Balance;
import frc.robot.commands.Intake.Intake.IntakeCube;
import frc.robot.commands.Intake.Launch.LaunchCube;
import frc.robot.commands.Intake.Outtake.OuttakeCubeAuto;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

/** Add your docs here. */
public class Routines {

    private final Drivetrain drivetrain;
    private final Arm arm;
    private final Intake intake;
    private final Macros macros;

    public Routines(Drivetrain drive, Intake intake, Arm ARM, Macros macros){
      this.intake = intake;
      this.drivetrain = drive;
      this.arm = ARM;
      this.macros = macros;
    }

    public CommandBase OutRightBalance(Drivetrain drivetrain){
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("OutRightBalance",4, 3);

      return Commands.sequence(
        new InstantCommand(intake::cInt, intake),
        baseSwerveCommand(trajectory, true),
        new Balance(drivetrain)
      );
    }

    public CommandBase OutLeftBalance(Drivetrain drivetrain){
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("OutLeftBalance",4, 3);

      return Commands.sequence(
        new InstantCommand(intake::cOut, intake),
        baseSwerveCommand(trajectory, true),
        new Balance(drivetrain)
      );
    }
    
    public CommandBase OutMidBalance(Drivetrain drivetrain){
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("OutMidBalance", 4, 3);

        return Commands.sequence(
          //new InstantCommand(intake::cOut, intake),
          baseSwerveCommand(trajectory, true),
          new Balance(drivetrain)
        );

    }

    public CommandBase goForward(Drivetrain drivetrain){
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("ShootPickup", 4, 3);

      HashMap<String,Command> eventMap = new HashMap<>();
          eventMap.put("OutCube", new LaunchCube(intake));  
      
      FollowPathWithEvents grabCubeAndDock = new FollowPathWithEvents(
        baseSwerveCommand(trajectory, true), 
        trajectory.getMarkers(), 
        eventMap);
          
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
      return Commands.sequence(
        //new InstantCommand(intake::cInt, intake),
        //grabCubeAndDock
        //new LaunchCube(intake),
        new IntakeCube(intake).withTimeout(0.4),
        new OuttakeCubeAuto(intake).withTimeout(1),
        baseSwerveCommand(trajectory, true),
        //new WaitCommand(3),
        macros.armIntake()
       // new Balance(drivetrain).withTimeout(5)
      );

  }

    public Command baseSwerveCommand(PathPlannerTrajectory trajectory, boolean isFirstPath) {
        InstantCommand resetOdom = new InstantCommand(() -> {
          if(isFirstPath) {
            drivetrain.resetOdometry(trajectory.getInitialHolonomicPose());
          }
        }, drivetrain);
    
    
        PPSwerveControllerCommand command = new PPSwerveControllerCommand(
          trajectory, 
          drivetrain::getPose, 
          drivetrain.getKinematics(), 
          new PIDController(4, 0, 0.02), 
          new PIDController(6.8, 0, 0.02), 
          new PIDController(0.7, 0, 0.02), 
          drivetrain::setModuleStates, 
          drivetrain);

          
    
        Timer timer = new Timer();
        SmartDashboard.putNumber("Desired X", 0);
              SmartDashboard.putNumber("Desired Y", 0);
              SmartDashboard.putNumber("Desired R", 0);
              SmartDashboard.putNumber("Error X", 0);
              SmartDashboard.putNumber("Error Y", 0);
              SmartDashboard.putNumber("Error R", 0);
    
        return Commands.sequence(
          resetOdom.andThen(Commands.runOnce(
            () -> {
              timer.reset();
              timer.start();
            })), 
          command.raceWith(
            Commands.run(() -> {
              double t = timer.get();
              PathPlannerState state = (PathPlannerState) trajectory.sample(t);
              SmartDashboard.putNumber("Desired X", state.poseMeters.getX());
              SmartDashboard.putNumber("Desired Y", state.poseMeters.getY());
              SmartDashboard.putNumber("Desired R", state.poseMeters.getRotation().getDegrees());
              SmartDashboard.putNumber("Error X", state.poseMeters.getX() - drivetrain.getPose().getX());
              SmartDashboard.putNumber("Error Y", state.poseMeters.getY() - drivetrain.getPose().getY());
              SmartDashboard.putNumber("Error R", state.poseMeters.getRotation().getDegrees() - drivetrain.getPose().getRotation().getDegrees());
              SmartDashboard.putNumber("currentXPose",drivetrain.getPose().getX() );
              SmartDashboard.putNumber("currentDegrees",drivetrain.getPose().getRotation().getDegrees());

            })
          ), 
          Commands.runOnce(() -> SmartDashboard.putString("Path Ended", "yes")));
      }

}
