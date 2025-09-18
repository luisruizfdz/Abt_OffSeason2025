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

import edu.wpi.first.math.controller.PIDController;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Cmd_AlgasIntake_PID;

public class Sub_IntakeAlgas extends SubsystemBase {
  /** Creates a new Sub_Motores. */
  private final SparkMax motor1 = new SparkMax(14, MotorType.kBrushless);
  private final SparkMaxConfig Config_Motor1 = new SparkMaxConfig();

  private final SparkMax ruedas = new SparkMax(15, MotorType.kBrushless);
  private final SparkMaxConfig Config_ruedas = new SparkMaxConfig();


  public boolean coral;

  public Sub_IntakeAlgas() {
    Config_Motor1.idleMode(IdleMode.kBrake); 
    motor1.configure (Config_Motor1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor1.getEncoder().setPosition(0);
  
    Config_ruedas.idleMode(IdleMode.kBrake); 
    ruedas.configure (Config_Motor1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    ruedas.getEncoder().setPosition(0);

   

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Algas Speed", motor1.get());
    SmartDashboard.putNumber("Algas Encoder", getEncoder());
    SmartDashboard.putNumber("Ruedas", getRuedasCurrent());


    //System.out.println(motor1.get());
    //System.out.println(motor1.getEncoder().getPosition()* 360/55.227);

    if(getRuedasCurrent()>=50){coral=true;}
    else{coral=false;}




  }
  public void setMotor1Speed(double speed) {
    motor1.set(speed);
  }
  public void setMotor_AlgaSpeed(double speed) {
  }
  public void setMotor_RuedasSpeed(double speed) {
    ruedas.set(speed);
  }
  public double getEncoder(){
    return motor1.getEncoder().getPosition()*1/40;
  }
  
  
  public double getRuedasCurrent(){
    return ruedas.getOutputCurrent();
  }
}