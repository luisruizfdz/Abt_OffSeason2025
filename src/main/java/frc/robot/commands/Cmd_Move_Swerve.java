// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Swerve;
//import frc.robot.subsystems.Sub_LEDs;
import frc.robot.subsystems.Sub_Swerve;
import java.util.function.Supplier;

public class Cmd_Move_Swerve extends Command {
  private final Sub_Swerve sub_Swerve;
 // private final Sub_LEDs Leds;
  private final Supplier<Double>  Xaxis,Yaxis,giros;
  private final Supplier<Boolean>fieldoriented,slow;
  private final Supplier<Boolean> fast;
  public Cmd_Move_Swerve(Sub_Swerve Sub_Swerve,Supplier<Double> Xaxis,Supplier<Double> Yaxis,Supplier<Double> giros,Supplier<Boolean> fieldoriented,Supplier<Boolean> slow, Supplier<Boolean> fast) {
    this.sub_Swerve=Sub_Swerve;
    this.Xaxis=Xaxis;
    this.Yaxis=Yaxis;
    this.giros=giros;
    this.fieldoriented=fieldoriented;
    this.slow=slow;
    this.fast=fast;

    addRequirements(Sub_Swerve);
  }

 
  @Override
  public void initialize() {
    sub_Swerve.zeroHeading();
  }
  ChassisSpeeds chassisSpeeds;
  @Override
  public void execute() {
    double velocidadx=Xaxis.get()*-1;
    double velocidady=(Yaxis.get())*-1;
    double velocidad_giros=giros.get()*-1;
    double fium;

    if (Math.abs(Xaxis.get())<0.07){velocidadx=0;}
    if (Math.abs(Yaxis.get())<0.07){velocidady=0;}
    if (Math.abs(giros.get())<0.07 ){velocidad_giros=0;}

    //antiboost
    if (slow.get()){
      fium=.7;
    }
    else{
      fium=.7;
    }

    //boost
    if (fast.get()){
      fium=0.7;
    } 
    

    if (fieldoriented.get()){
      
      chassisSpeeds= new ChassisSpeeds(velocidady*fium,velocidadx*fium, velocidad_giros*fium); 
    }
    else{
      chassisSpeeds= ChassisSpeeds.fromFieldRelativeSpeeds(velocidady*fium, velocidadx*fium, velocidad_giros*fium, sub_Swerve.get2Drotation());
    }
    //Manda un arreglo de estados de modulos que pasan por un objeto de Swerve drive kinematics para poder generar las velocidades
    SwerveModuleState[] moduleStates=Swerve.swervekinematics.toSwerveModuleStates(chassisSpeeds);
    sub_Swerve.setModuleStates(moduleStates);
  }


  @Override
  public void end(boolean interrupted) {
    sub_Swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}