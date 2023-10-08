// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.commands.Intake.Intake.IntakeCube;
import frc.robot.commands.Intake.Launch.LaunchCube;
import frc.robot.commands.Intake.Outtake.OuttakeCube;
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

    public CommandBase MidDock(Drivetrain drivetrain){
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("MidDock",4, 3);

      return Commands.sequence(
        new IntakeCube(intake).withTimeout(0.4),
        new OuttakeCubeAuto(intake).withTimeout(1),
        baseSwerveCommand(trajectory, true),
        new Balance(drivetrain)
      );
    }

    public CommandBase backwardsStraight(Drivetrain drivetrain){
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("BackwardsStraight", 4, 3);
  
      return Commands.sequence(
        new IntakeCube(intake).withTimeout(0.4),
        new OuttakeCubeAuto(intake).withTimeout(1),
        baseSwerveCommand(trajectory, true)
      );
  }

    //FIX TURNING BEFORE USING (CLEAR CHARGE STATION TRAJ)
    public CommandBase backwardsLeftClear(Drivetrain drivetrain){
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("BackwardsPathLeft",4, 3);

      return Commands.sequence(
        new IntakeCube(intake).withTimeout(0.4),
        new OuttakeCubeAuto(intake).withTimeout(1),
        baseSwerveCommand(trajectory, true)
      );
    }

    //FIX TURNING BEFORE USING (CLEAR CHARGE STATION TRAJ)
    public CommandBase backwardsRightClear(Drivetrain drivetrain){
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("BackwardsPathRight",4, 3);

      return Commands.sequence(
        new IntakeCube(intake).withTimeout(0.4),
        new OuttakeCubeAuto(intake).withTimeout(1),
        baseSwerveCommand(trajectory, true)
      );
    }
    
    public CommandBase shootPickLeft(Drivetrain drivetrain){
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("ShootPickupLeft", 4, 3);

      return Commands.sequence(
        new IntakeCube(intake).withTimeout(0.4),
        new OuttakeCubeAuto(intake).withTimeout(1),
        baseSwerveCommand(trajectory, true)
      );
  }

    public CommandBase shootPickRight(Drivetrain drivetrain){
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("ShootPickupRight", 4, 3);

      return Commands.sequence(
        new IntakeCube(intake).withTimeout(0.4),
        new OuttakeCubeAuto(intake).withTimeout(1),
        baseSwerveCommand(trajectory, true)
      );
  }
    public CommandBase shootPickRightCombo(Drivetrain drivetrain){
      PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("ShootPickRightCombo", 4, 3);
      PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("Turn90Counter", 1, 0.5);

      return Commands.sequence(
        new IntakeCube(intake).withTimeout(0.4),
        new OuttakeCubeAuto(intake).withTimeout(1),
        baseSwerveCommand(trajectory1, true),
        baseSwerveCommand(trajectory2, false),
        macros.armIntake()
      );
  }

  public CommandBase shootPickLeftCombo(Drivetrain drivetrain){
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("ShootPickLeftCombo", 4, 3);
    PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("Turn90Counter", 1, 0.5);

    return Commands.sequence(
      new IntakeCube(intake).withTimeout(0.4),
      new OuttakeCubeAuto(intake).withTimeout(1),
      baseSwerveCommand(trajectory1, true),
      baseSwerveCommand(trajectory2, false),
      macros.armIntake()
    );
}

    public CommandBase goForward(Drivetrain drivetrain){
      PathPlannerTrajectory trajectory = PathPlanner.loadPath("GoForwardTwo", 4, 3);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
      return Commands.sequence(
        baseSwerveCommand(trajectory, true)
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
