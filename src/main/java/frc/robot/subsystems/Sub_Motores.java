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
//import frc.robot.commands.cmd_Move_Motor1;

public class Sub_Motores extends SubsystemBase {
  /** Creates a new Sub_Motores. */
  private final SparkMax motor1 = new SparkMax(15, MotorType.kBrushless);
  private final SparkMaxConfig Config_Motor1 = new SparkMaxConfig();


  public Sub_Motores() {
    Config_Motor1.idleMode(IdleMode.kBrake); 
    motor1.configure (Config_Motor1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Motor1 Speed", motor1.get());
    System.out.println(motor1.get());

  }
  public void setMotor1Speed(double speed) {
    motor1.set(speed);
  }
}

