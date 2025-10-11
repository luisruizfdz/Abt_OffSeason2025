// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsistemas;

public class Sub_Climber extends SubsystemBase {
 
  //Motor NEO Mini
  private final SparkMax motorRollersClimber = new SparkMax(20, MotorType.kBrushless);
  private final SparkMaxConfig Config_motorRuedasClimber = new SparkMaxConfig();

  //Motor NEO 
  private final SparkMax motorMoveClimber = new SparkMax(21, MotorType.kBrushless);
  private final SparkMaxConfig Config_motorMoveClimber = new SparkMaxConfig();

  public boolean grabbed;

  public Sub_Climber() {
    Config_motorRuedasClimber.idleMode(IdleMode.kBrake); 
    motorRollersClimber.configure (Config_motorRuedasClimber, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorRollersClimber.getEncoder().setPosition(0);

    Config_motorMoveClimber.idleMode(IdleMode.kBrake); 
    motorMoveClimber.configure (Config_motorMoveClimber, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorMoveClimber.getEncoder().setPosition(0);
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //SmartDashboard.putNumber("MoveClimber Speed", motorMoveClimber.get());
    //SmartDashboard.putNumber("MoveClimber Encoder", getMotorMoveClimberEncoder());
    //SmartDashboard.putNumber("RollersClimber Current", getMotorRollersClimberCurrent());
  
    if(getMotorRollersClimberCurrent()>=50){grabbed=true;}
    else{grabbed=false;}
  }
  
  
  public void setMotorMoveClimberSpeed(double speed) {
    motorMoveClimber.set(speed);
  }
  
  public double getMotorMoveClimberEncoder(){
    return motorMoveClimber.getEncoder().getPosition()*Subsistemas.conversion_Climber; //(9:1), (5:1), (3:1)
  }

  public double getMotorRollersClimberCurrent(){
    return motorRollersClimber.getOutputCurrent();
  }

  public void setMotorRollersClimberSpeed(double speed) {
    motorRollersClimber.set(speed);
  }
  
  public double getMotorRollersClimberEncoder(){
    return motorRollersClimber.getEncoder().getPosition(); 

}
}
