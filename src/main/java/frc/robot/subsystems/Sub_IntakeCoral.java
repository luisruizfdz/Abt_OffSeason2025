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
import frc.robot.Constants.Subsistemas;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Sub_IntakeCoral extends SubsystemBase {

  //Motor NEO Vortex
  private final SparkFlex motorBrazoCoral = new SparkFlex(14, MotorType.kBrushless);
  private final SparkFlexConfig Config_motorBrazoCoral = new SparkFlexConfig();

  //Motor NEO Vortex
  private final SparkFlex motorEstrellas = new SparkFlex(15, MotorType.kBrushless);
  private final SparkFlexConfig Config_estrellas = new SparkFlexConfig();



  public Sub_IntakeCoral() {
    Config_motorBrazoCoral.idleMode(IdleMode.kBrake); 
    motorBrazoCoral.configure (Config_motorBrazoCoral, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorBrazoCoral.getEncoder().setPosition(0);
  
    Config_estrellas.idleMode(IdleMode.kBrake); 
    motorEstrellas.configure (Config_estrellas, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorEstrellas.getEncoder().setPosition(0);

   

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
    //SmartDashboard.putNumber("Intake Speed", motorBrazoCoral.get());
    //SmartDashboard.putNumber("Estrellas Encoder", getEncoderBrazoCoral());


    //System.out.println(motor1.get());
    //System.out.println(motor1.getEncoder().getPosition()* 360/55.227);

    
    
  }
  

  public void setMotor_EstrellasSpeed(double speed) {
    motorEstrellas.set(speed);
  }
  public void setMotor_IntakeSpeed(double speed) {
    motorBrazoCoral.set(speed);
  }

  public double getEncoderBrazoCoral(){
    return motorBrazoCoral.getEncoder().getPosition()*Subsistemas.conversion_BrazoIntake;
  }
  
  public double getEstrellasCurrent(){
    return motorEstrellas.getOutputCurrent();
  }
}
  
