// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnToAngle extends CommandBase {
  private final Drivetrain drivetrain;

  
  private final PIDController m_turningPIDController = new PIDController(SwerveConstants.ProfiledPIDp, SwerveConstants.ProfiledPIDi, SwerveConstants.ProfiledPIDd);

  // private final ProfiledPIDController m_turningPIDController =
  // new ProfiledPIDController(
  //     SwerveConstants.ProfiledPIDp,
  //     SwerveConstants.ProfiledPIDi,
  //     SwerveConstants.ProfiledPIDd,
  //     new TrapezoidProfile.Constraints(
  //         SwerveConstants.kModuleMaxAngularVelocity, SwerveConstants.kModuleMaxAngularAcceleration));

  private final double targetAngle;
  private final double turnTolerance = 0.02;
  private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  /** Creates a new TurnToAngle. */
  public TurnToAngle(Drivetrain drivetrain, double targetAngle) {
    this.drivetrain = drivetrain;
    this.targetAngle = targetAngle;

    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turningPIDController.reset();
    ahrs.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = ahrs.getYaw();
    double pidOutput = m_turningPIDController.calculate(currentAngle, targetAngle);
    drivetrain.drive(0, 0, pidOutput, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0,0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(targetAngle - drivetrain.getYaw()) < 0;
  }
}
