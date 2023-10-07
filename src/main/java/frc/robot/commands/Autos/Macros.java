// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;

/** Add your docs here. */
public class Macros {
    private Arm arm;
    private boolean isFinished;

    public Macros(Arm arm){
        this.arm = arm;
    }

    public CommandBase armIntake(){
        arm.armDown();
        Commands.runOnce(() -> new WaitCommand(5));
        return Commands.sequence();
    }

}
