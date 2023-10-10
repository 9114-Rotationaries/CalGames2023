// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Forklift;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Forklift;

public class LowerForklift extends CommandBase {

  private static Forklift forklift;

  /** Creates a new RaiseForklift. */
  public LowerForklift(Forklift forklift) {
    this.forklift = forklift;
    addRequirements(forklift);
  }

  @Override
  public void initialize() {
    forklift.stop();
  }

  @Override
  public void execute() {
    forklift.moveForklift(0.9);
  }

  @Override
  public void end(boolean interrupted) {
    forklift.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}