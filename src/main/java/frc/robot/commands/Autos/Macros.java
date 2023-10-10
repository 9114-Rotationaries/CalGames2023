package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.Arm.LowerArm;
import frc.robot.commands.Arm.RaiseArm;
import frc.robot.commands.Intake.Intake.IntakeCube;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

/** Add your docs here. */
public class Macros {
    private Arm arm;
    private Intake intake;
    private boolean isFinished;

    public Macros(Arm arm, Intake intake){
        this.arm = arm;
        this.intake = intake;
    }

    public CommandBase armIntake(){
        return Commands.sequence(
            new LowerArm(arm).withTimeout(0.95),
            new IntakeCube(intake).withTimeout(1),
            new RaiseArm(arm).withTimeout(0.95)
        );
    }

}
