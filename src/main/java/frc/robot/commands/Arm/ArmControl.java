// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmControl extends CommandBase {
  XboxController controller;
  Arm arm;
  double armDegree = 0;
  
  /** Creates a new ArmControl. */
  public ArmControl(XboxController controller, Arm arm) {
    this.controller = controller;
    this.arm = arm;
    addRequirements(arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(controller.getRightTriggerAxis() > 0.1){
      armDegree += controller.getRightTriggerAxis();
    }

    if(controller.getLeftTriggerAxis() > 0.1){
      armDegree -= controller.getLeftTriggerAxis();
    }

    if(controller.getRightTriggerAxis() <= 0.1 && controller.getLeftTriggerAxis() <= 0.1){
      arm.stop();
    }

    arm.pivot(armDegree);
    SmartDashboard.putNumber("Arm Angle", arm.getAngle());
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
