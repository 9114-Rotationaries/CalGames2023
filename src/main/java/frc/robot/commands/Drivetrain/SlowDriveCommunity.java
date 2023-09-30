// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class SlowDriveCommunity extends CommandBase {
  CommandXboxController controller;
  Drivetrain drive;
  boolean fieldRelative;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(1.25);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(1.25);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(1.25);

  public SlowDriveCommunity(CommandXboxController controller, Drivetrain drive, boolean fieldRelative) {
    this.controller = controller;
    this.drive=drive;
    this.fieldRelative = fieldRelative;
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Inverted X speed / Forwards and backwards
    final double xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.02))
            * DriveConstants.kCommunityAngularSpeed;

    // Inverted Y speed / Strafe speed
    final double ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.02))
            * DriveConstants.kCommunitySpeed;

    // Inverted angular rotation value
    double rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), 0.02))
            * DriveConstants.kCommunityAngularSpeed;
  
      
      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("Rotation", rot);

    drive.drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
