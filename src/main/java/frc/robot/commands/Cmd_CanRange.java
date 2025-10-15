// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.core.CoreCANrange;

import com.ctre.phoenix6.hardware.*; 


import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_CanRange extends Command {
  public CoreCANrange canRange = new CoreCANrange(25);
  double distancia; 
  boolean objetoDetectado; 
  public boolean coral;  

  public Cmd_CanRange() {
    // Use addRequirements() here to declare subsystem dependencies.
   // this.canRange = caNrange;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coral = canRange.getIsDetected().getValue();

    distancia = canRange.getDistance().getValueAsDouble(); 

    if(distancia<=0.07){
      objetoDetectado= true; 
    }
    else{
      objetoDetectado= false;
    }

    if(objetoDetectado){
      System.out.println("Funciona");
    }
    else{
      System.out.println("No funciona");
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
