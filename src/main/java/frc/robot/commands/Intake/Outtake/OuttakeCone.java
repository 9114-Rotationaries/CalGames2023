package frc.robot.commands.Intake.Outtake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class OuttakeCone extends CommandBase {
  private Intake intake;
  /** Creates a new OuttakeObject. */
  public OuttakeCone(Intake intake) {
    this.intake=intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // intake.coneOuttake(IntakeConstants.coneOuttakeSpeed);
    // intake.cubeIntake(-IntakeConstants.coneOuttakeSpeed)
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
