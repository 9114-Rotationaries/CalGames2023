package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends CommandBase {
  CommandXboxController controller;
  Drivetrain drive;
  boolean fieldRelative;

  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);

  public JoystickDrive(CommandXboxController controller, Drivetrain drive, boolean fieldRelative) {
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
            * DriveConstants.kMaxSpeed;

    // Inverted Y speed / Strafe speed
    final double ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.02))
            * DriveConstants.kMaxSpeed;

    // Inverted angular rotation value
    double rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), 0.02))
            * DriveConstants.kMaxAngularSpeed;
  
      
      //SmartDashboard.putNumber("xSpeed", xSpeed);
      //SmartDashboard.putNumber("ySpeed", ySpeed);
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
