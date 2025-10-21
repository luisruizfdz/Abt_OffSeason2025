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
  private final SparkMax motorCableClimber = new SparkMax(36, MotorType.kBrushless);
  private final SparkMaxConfig Config_motorCableClimber = new SparkMaxConfig();

  //Motor NEO 
  private final SparkMax motorLlantaClimber = new SparkMax(32, MotorType.kBrushless);
  private final SparkMaxConfig Config_motorLlantaClimber = new SparkMaxConfig();

  public boolean grabbed;

  public Sub_Climber() {
    Config_motorCableClimber.idleMode(IdleMode.kBrake); 
    motorCableClimber.configure (Config_motorCableClimber, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorCableClimber.getEncoder().setPosition(0);

    Config_motorLlantaClimber.idleMode(IdleMode.kBrake); 
    motorLlantaClimber.configure (Config_motorLlantaClimber, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorLlantaClimber.getEncoder().setPosition(0);
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //SmartDashboard.putNumber("MoveClimber Speed", motorMoveClimber.get());
    SmartDashboard.putNumber("MoveClimber Encoder", getMotorCableClimberEncoder());
    //SmartDashboard.putNumber("RollersClimber Current", getMotorRollersClimberCurrent());
  
    if(getMotorCableClimberCurrent()>=50){grabbed=true;}
    else{grabbed=false;}
  }
  
  
  public void setMotorCableClimberSpeed(double speed) {
   
    motorCableClimber.set(speed);
    
  }
  public double getMotorCableClimberEncoder(){
    
    return motorCableClimber.getEncoder().getPosition()*Subsistemas.conversion_Climber; //(9:1), (5:1), (3:1)
  }
  
  public double getMotorLlantaClimberEncoder(){
    return motorLlantaClimber.getEncoder().getPosition()*Subsistemas.conversion_Climber; //(9:1), (5:1), (3:1)
  }

  public double getMotorCableClimberCurrent(){
    return motorCableClimber.getOutputCurrent();
  }

  public void setMotorLlantaClimberSpeed(double speed) {
    motorLlantaClimber.set(speed);
  }
  
}
