// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Subsistemas;

public class Sub_Elevador extends SubsystemBase {

  //Motor NEO Vortex
  private final SparkFlex motorElevadorL = new SparkFlex(16, MotorType.kBrushless);
  private final SparkFlexConfig Config_motorElevadorL = new SparkFlexConfig();

  //Motor NEO Vortex
  private final SparkFlex motorElevadorR = new SparkFlex(17, MotorType.kBrushless);
  private final SparkFlexConfig Config_motorElevadorR = new SparkFlexConfig();


  public Sub_Elevador() {
    Config_motorElevadorL.idleMode(IdleMode.kBrake); 
    motorElevadorL.configure (Config_motorElevadorL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorElevadorL.getEncoder().setPosition(0);
    Config_motorElevadorL.inverted(false);
  
    Config_motorElevadorR.idleMode(IdleMode.kBrake); 
    motorElevadorR.configure (Config_motorElevadorR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorElevadorR.getEncoder().setPosition(0);
    Config_motorElevadorR.inverted(true);


   

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("L_Elevador Speed", motorElevadorL.get());
    //SmartDashboard.putNumber("L_Elevador Encoder", getL_ElevadorEncoder());
    //SmartDashboard.putNumber("R_Elevador Speed", motorElevadorR.get());
    //SmartDashboard.putNumber("R_Elevador Encoder", getR_ElevadorEncoder());
    
    


    //System.out.println(motor1.get());
    //System.out.println(motor1.getEncoder().getPosition()* 360/55.227);

    
    
  }
  public void setL_ElevadorSpeed(double speed) {
    motorElevadorL.set(speed);
  }

  public void setR_ElevadorSpeed(double speed) {
    motorElevadorR.set(speed);
  }

  public double getL_ElevadorEncoder(){
    return motorElevadorL.getEncoder().getPosition()*Subsistemas.conversion_Elevador;
  }

  public double getR_ElevadorEncoder(){
    return motorElevadorR.getEncoder().getPosition()*Subsistemas.conversion_Elevador;
  }
  
  }

  