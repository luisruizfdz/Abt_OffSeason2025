// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/////////////////////////////////////// Mover las ruedas del indexer ///////////////////////////////////////////

package frc.robot.commands;

import frc.robot.subsystems.*; 
import edu.wpi.first.wpilibj2.command.Command;

//import edu.wpi.first.math.controller.PIDController; 



public class Cmd_ClimberCable extends Command {

  private final Sub_Climber climber;
  double grabbed;

  public Cmd_ClimberCable(Sub_Climber climber){

    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
      climber.setMotorCableClimberSpeed(0.9);
      
    }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setMotorCableClimberSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    /*if(climber.grabbed==true){
      return true;
    }
    else{return false;} **/ 

    return false; 
  }
}
