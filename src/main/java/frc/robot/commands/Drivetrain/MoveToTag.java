// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class MoveToTag extends CommandBase {

  private Vision vision;
  private Drivetrain swerveDrive;

  private PIDController angleController = new PIDController(VisionConstants.VAnglePIDp, VisionConstants.VAnglePIDi, VisionConstants.VAnglePIDd);
  private PIDController distanceController = new PIDController(VisionConstants.VDrivePIDp, VisionConstants.VDrivePIDi, VisionConstants.VDrivePIDd);
  private PIDController rotationController = new PIDController(VisionConstants.ProfiledVRotPIDp, VisionConstants.ProfiledVRotPIDi, VisionConstants.ProfiledVRotPIDd);

  /** Creates a new MoveToTag. */
  public MoveToTag(Vision vision, Drivetrain swerveDrive) {
    this.vision = vision;
    this.swerveDrive = swerveDrive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleController.setSetpoint(VisionConstants.alignSetpoint); //Align with Target
    distanceController.setSetpoint(VisionConstants.driveSetpoint); //How far away
    rotationController.setSetpoint(VisionConstants.rotSetpoint); //Oriented

    angleController.setTolerance(VisionConstants.angleTolerance);
    distanceController.setTolerance(VisionConstants.distanceTolerance);
    rotationController.setTolerance(VisionConstants.rotTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angleError = vision.getTX();
    double distanceError = vision.getDistance() - distanceController.getSetpoint();
    double rotationError = vision.getTV();

    double angleCorrection = angleController.calculate(angleError);
    double distanceCorrection = distanceController.calculate(distanceError);
    double rotationCorrection = rotationController.calculate(rotationError);

    swerveDrive.drive(-distanceCorrection, angleCorrection, rotationCorrection, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return angleController.atSetpoint() && distanceController.atSetpoint() && rotationController.atSetpoint();
  }
}
