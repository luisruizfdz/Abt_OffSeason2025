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

public class Sub_Indexer extends SubsystemBase {

  //Motor NEO 
  private final SparkMax motorIndexer = new SparkMax(19, MotorType.kBrushless);
  private final SparkMaxConfig Config_Indexer = new SparkMaxConfig();


  public Sub_Indexer() {
    Config_Indexer.idleMode(IdleMode.kBrake); 
    motorIndexer.configure (Config_Indexer, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorIndexer.getEncoder().setPosition(0);
  

  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    //SmartDashboard.putNumber("Indexer Speed", motorIndexer.get());
    //SmartDashboard.putNumber("Indexer Encoder", getIndexerEncoder());

  }

    //System.out.println(motor1.get());
    //System.out.println(motor1.getEncoder().getPosition()* 360/55.227);

  
  public void setMotorIndexerSpeed(double speed) {
    motorIndexer.set(speed);
  }
  public double getIndexerEncoder(){
    return motorIndexer.getEncoder().getPosition();
  }
  
}


