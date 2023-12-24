// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class FollowTag extends CommandBase {
  private final Drivetrain drive;
  private final Vision visionSubsystem;
  private PIDController horizontalOffsetController;
  private PIDController orientationController;
  private PIDController distanceController;

  /** Creates a new FollowTag. */
  public FollowTag(Drivetrain drive, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.visionSubsystem = vision;
    this.drive = drive;

    distanceController = new PIDController(0.2,0,0.01);
    horizontalOffsetController = new PIDController(0.1, 0.0, 0.01); // Set your PID constants
    orientationController = new PIDController(0.05, 0, 0.015);
        
    addRequirements(vision, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    horizontalOffsetController.setSetpoint(0);
    orientationController.setSetpoint(0);
    distanceController.setSetpoint(0);

        // Configure tolerances for the controllers
    horizontalOffsetController.setTolerance(0.02);
    orientationController.setTolerance(0.02);
    distanceController.setTolerance(0.02);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHorizontalOffset = visionSubsystem.getHorizontalOffset();
        double currentOrientation = visionSubsystem.getOrientation();
        double currentDistance = visionSubsystem.getDistance();

        // Calculate control outputs
        double horizontalOffsetOutput = horizontalOffsetController.calculate(currentHorizontalOffset);
        double orientationOutput = orientationController.calculate(currentOrientation);
        double distanceOutput = distanceController.calculate(currentDistance);
        
        while(visionSubsystem.isTargetValid()){
          SmartDashboard.putNumber("horizontalOffsetOutput", horizontalOffsetOutput);
          SmartDashboard.putNumber("distanceOutput", distanceOutput);
          SmartDashboard.putNumber("orientationOutput", orientationOutput);
          
          drive.drive(horizontalOffsetOutput, distanceOutput, orientationOutput, false);
        } if (!visionSubsystem.isTargetValid()){
          drive.drive(0,0,0, false);
        }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
