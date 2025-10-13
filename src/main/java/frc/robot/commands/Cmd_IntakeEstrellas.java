// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/////////////////////////////////////// Mover las ruedas del intake de corales ///////////////////////////////////////////


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_IntakeCoral;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_IntakeEstrellas extends Command {
  /** Creates a new Cmd_Move. */
  private final Sub_IntakeCoral Motor;

  public Cmd_IntakeEstrellas(Sub_IntakeCoral Motores){
    // Use addRequirements() here to declare subsystem dependencies.
    this.Motor = Motores;
    addRequirements(Motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      Motor.setMotor_EstrellasSpeed(-.5);
      
    }
  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Motor.setMotor_EstrellasSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
