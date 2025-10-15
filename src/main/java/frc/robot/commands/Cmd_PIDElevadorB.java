// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class Cmd_PIDElevadorB extends Command {

  private final Sub_Elevador SubM;

  public Cmd_PIDElevadorB(Sub_Elevador SubM) {

    this.SubM = SubM;
    addRequirements(SubM);


  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    SubM.setL_ElevadorSpeed(0.3);
    SubM.setR_ElevadorSpeed(-0.3);



  }

  @Override
  public void end(boolean interrupted) {

    SubM.setL_ElevadorSpeed(0);
    SubM.setR_ElevadorSpeed(0);


  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
