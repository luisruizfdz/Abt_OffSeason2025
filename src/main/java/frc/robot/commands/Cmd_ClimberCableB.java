// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

public class Cmd_ClimberCableB extends Command {

  private Sub_Climber climber; 

  public Cmd_ClimberCableB(Sub_Climber climber) {
    this.climber= climber; 
    addRequirements(climber);

  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    climber.setMotorCableClimberSpeed(-0.5);


  }

  @Override
  public void end(boolean interrupted) {

    climber.setMotorCableClimberSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
