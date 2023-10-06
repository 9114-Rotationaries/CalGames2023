// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

/** Add your docs here. */
public class Routines {

    private final Drivetrain drivetrain = new Drivetrain();

    public Routines(){}

    public CommandBase goForward(){
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("Go Forward", 4, 3);

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
          new PIDController(0.225, 0, 0.004225), 
          new PIDController(0.255, 0, 0.004225), 
          new PIDController(0.6, 0, 0), 
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
              SmartDashboard.putNumber("Desired R", state.poseMeters.getRotation().getRadians());
              SmartDashboard.putNumber("Error X", state.poseMeters.getX() - drivetrain.getPose().getX());
              SmartDashboard.putNumber("Error Y", state.poseMeters.getY() - drivetrain.getPose().getY());
              SmartDashboard.putNumber("Error R", state.poseMeters.getRotation().getRadians() - drivetrain.getPose().getRotation().getRadians());
            })
          ), 
          Commands.runOnce(() -> SmartDashboard.putString("Path Ended", "yes")));
      }

}
