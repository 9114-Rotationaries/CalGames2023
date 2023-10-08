// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
            new LowerArm(arm).withTimeout(1),
            new IntakeCube(intake).withTimeout(1),
            new RaiseArm(arm).withTimeout(1)
        );
    }

}
