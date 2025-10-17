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

public class Sub_EndEffector extends SubsystemBase {

  //Motor NEO Mini
  private final SparkMax motorEndEffector = new SparkMax(33, MotorType.kBrushless);
  private final SparkMaxConfig Config_EndEffector = new SparkMaxConfig();

  public boolean coral;

  public Sub_EndEffector() {
    
    Config_EndEffector.idleMode(IdleMode.kBrake); 
    motorEndEffector.configure (Config_EndEffector, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorEndEffector.getEncoder().setPosition(0);
  

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("EndEffector Speed", motorEndEffector.get());
    //SmartDashboard.putNumber("EndEffector Encoder", getEndEffectorEncoder());
    //SmartDashboard.putNumber("EndEffector Current", getEndEffectorCurrent());

    /* 
    if(getEndEffectorCurrent()>=50){coral=true;}
    else{coral=false;}
  
  */  }

    //System.out.println(motor1.get());
    //System.out.println(motor1.getEncoder().getPosition()* 360/55.227);

  
  public void setMotorEndEffectorSpeed(double speed) {
    motorEndEffector.set(speed);
  }


  public double getEndEffectorEncoder(){
    return motorEndEffector.getEncoder().getPosition()*Subsistemas.conversion_EndEffector;
  }
  public double getEndEffectorCurrent(){
    return motorEndEffector.getOutputCurrent();
  }

}
