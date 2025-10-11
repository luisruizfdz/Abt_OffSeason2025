// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/////////////////////////////////////// Elevador ///////////////////////////////////////////

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Elevador;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_PIDElevador extends Command {
  private final Sub_Elevador SubM;
  private final double setPoint;
  double last_time;
  double kP;
  double kI;
  double dt;
  double error;
  double integral_zone;
  double output;
  double error_i;
  double speed;
  double error_d;
  double kD;
  double last_error;

  /** Creates a new PID. */
  public Cmd_PIDElevador(Sub_Elevador SubM, double setPoint) {
    this.SubM = SubM;
    this.setPoint = setPoint;    
   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    //double Encoder = SubM.getEncoderCoral();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = setPoint - SubM.getL_ElevadorEncoder();
    integral_zone = setPoint*0.1; //10% del error aprox

    kP = 0.05;
    kI = 0.0;
    kD = 0.0;

    dt = Timer.getFPGATimestamp()-last_time;
    speed = error * kP + error_i * kI + error_d * kD;
    System.out.println(error);
    error_d = (error - last_error)/dt;

    if (Math.abs(error)<integral_zone){error_i+=error*dt;}

    SubM.setL_ElevadorSpeed(speed);
    SubM.setR_ElevadorSpeed(speed);

    last_time = Timer.getFPGATimestamp();
    last_error = error ;

  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SubM.setL_ElevadorSpeed(0);
    SubM.setR_ElevadorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  if (error<.7){
      return true;
    }
    else{
  
      return false;
  }
}
}
