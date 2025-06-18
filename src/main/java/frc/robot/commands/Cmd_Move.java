// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Sub_Motores;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Cmd_Move extends Command {
  /** Creates a new Cmd_Move. */
  private final Sub_Motores Motor;
  private final Supplier<Boolean> A;

  public Cmd_Move(Sub_Motores Motores, Supplier<Boolean> a){
    // Use addRequirements() here to declare subsystem dependencies.
    this.Motor = Motores;
    this.A= a;
    addRequirements(Motor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(A.get()){
      Motor.setMotor1Speed(.4);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Motor.setMotor1Speed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
