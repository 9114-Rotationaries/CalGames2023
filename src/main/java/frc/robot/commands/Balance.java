package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Balance extends CommandBase {
  /** Creates a new Balance. */
  Drivetrain m_drive;
  PIDController controller = new PIDController(0.05, 0, 0);

  public Balance(Drivetrain m_drive) {
    this.m_drive = m_drive;
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = controller.calculate(m_drive.getPitch());
    xSpeed = -MathUtil.clamp(xSpeed,-1,1);

    double ySpeed = controller.calculate(m_drive.getYaw());
    ySpeed = -MathUtil.clamp(ySpeed, -1, 1);

    m_drive.drive(xSpeed, ySpeed, 0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint();
  }
}