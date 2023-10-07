// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.Launch;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class LaunchCube extends CommandBase {
  private Intake intake;
  private boolean finishedFull;
  private boolean finishedIntake;
  private Timer timer = new Timer();
  /** Creates a new LaunchObject. */
  public LaunchCube(Intake intake) {
    this.intake=intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finishedFull = false;
    finishedIntake = false;
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("timer", timer.get());
    intake.cubeIntake(IntakeConstants.cubeIntakeSpeed);
    if (timer.get() > 5) {
      finishedIntake = true;
      intake.cubeOuttake(IntakeConstants.cubeOuttakeSpeed);
    }
    if (timer.get() < 10) {
      intake.stop();
      finishedFull = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finishedFull;
  }
}
