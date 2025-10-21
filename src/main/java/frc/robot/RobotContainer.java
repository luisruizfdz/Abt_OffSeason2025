// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.commands.*;
import frc.robot.subsystems.Sub_Elevador;
import frc.robot.subsystems.Sub_EndEffector;
import frc.robot.subsystems.Sub_Indexer;
import frc.robot.subsystems.Sub_IntakeCoral;
import frc.robot.subsystems.Sub_Swerve;

import frc.robot.subsystems.*;

import java.nio.file.Path;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Sub_IntakeCoral IntakeCoral = new Sub_IntakeCoral();
  private final Sub_Indexer Indexer = new Sub_Indexer();
  private final Sub_Elevador SubM = new Sub_Elevador();
  private final Sub_Climber climber= new Sub_Climber();
  private final Sub_EndEffector EndEffector = new Sub_EndEffector(); 
  //private final Sub_Indexer Indexer = new Sub_Indexer();
  private final Sub_Swerve swerve = new  Sub_Swerve();
 // private final Sub_Elevador Elevador = new Sub_Elevador();
 // private final Sub_EndEffector EndEffector = new Sub_EndEffector();


  // Replace with CommandPS4Controller or CommandJoystick if needed
  
    private final CommandXboxController joysubs = new CommandXboxController(1);
    private final CommandXboxController joydrive = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    NamedCommands.registerCommand("Vision derecha", new Cmd_AutoAlign(true, swerve));
    NamedCommands.registerCommand("Vision izquierda", new Cmd_AutoAlign(false, swerve));

    NamedCommands.registerCommand("Intake Coral rollers", new Cmd_IntakeEstrellas(IntakeCoral));
    //NamedCommands.registerCommand("PID Intake", new Cmd_PIDIntake(IntakeCoral, 0));

    NamedCommands.registerCommand("null", getAutonomousCommand());

    swerve.setDefaultCommand(new Cmd_Move_Swerve(swerve, () -> joydrive.getLeftX(), () -> joydrive.getLeftY(), ()-> joydrive.getRightX(),() -> joydrive.rightTrigger().getAsBoolean(),() -> joydrive.x().getAsBoolean(),() -> joydrive.b().getAsBoolean()));
  ;


    configureBindings();
    
  }

  
  private void configureBindings() {

    joydrive.back().whileTrue(new Cmd_zeroheding(swerve));
    
    joydrive.leftBumper().whileTrue(new Cmd_AutoAlign(false, swerve));
    joydrive.rightBumper().whileTrue(new Cmd_AutoAlign(true, swerve));

     //Climber 
    joydrive.y().whileTrue(new Cmd_ClimberCable(climber));
    joydrive.a().whileTrue(new Cmd_ClimberCableB(climber));
    joydrive.start().whileTrue(new Cmd_ClimberLlanta(climber)); 
 

   //Normal
      /*joysubs.y().whileTrue(new SequentialCommandGroup(new ParallelCommandGroup(
      new Cmd_IntakeEstrellas(IntakeCoral), 
      new Cmd_IndexerRollers(Indexer), 
      new Cmd_EndEffector(EndEffector)))); **/

      joysubs.y().whileTrue(new Cmd_IntakeEstrellas(IntakeCoral));


    //Reversed
      /*joysubs.a().whileTrue(new SequentialCommandGroup(new ParallelCommandGroup(
      new Cmd_ReversedIntake(IntakeCoral), 
      new Cmd_ReversedIndexer(Indexer), 
      new Cmd_ReversedEndEffector(EndEffector)))); **/ 

      joysubs.a().whileTrue(new Cmd_ReversedIntake(IntakeCoral));


    //joysubs.x().whileTrue(new Cmd_IntakeEstrellas(IntakeCoral));
    //joysubs.b().whileTrue(new Cmd_IndexerRollers(Indexer));
    //joysubs.a().whileTrue(new Cmd_EndEffector(EndEffector));


    //Intake 
    joysubs.b().whileTrue(new Cmd_PIDIntake(IntakeCoral, 1.20));
    joysubs.x().whileTrue(new Cmd_ReversedPIDIntake(IntakeCoral, 0));


    //Elevador
    joysubs.rightTrigger().whileTrue(new Cmd_PIDElevador(SubM));
    joysubs.leftTrigger().whileTrue(new Cmd_PIDElevadorB(SubM));

   
      
      
    // Schedule ExampleCommand when exampleCondition changes to true
  

    // Schedule exampleMethodCommand when the Xbox controller's B button is pressed,
    // cancelling on release.
   
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return new PathPlannerAuto("2--1");

//Rutina 1
/* 
return new SequentialCommandGroup (
  new PathPlannerAuto("1--1"), (new Cmd_AutoAlign(false, swerve)), (
  new PathPlannerAuto("1--coral")), (
  new PathPlannerAuto("1--3")), (new Cmd_AutoAlign(false, swerve)));
 */
//Rutina 2


//Rutina 3
/* 
return new SequentialCommandGroup (
  new PathPlannerAuto("3--1"), (new Cmd_AutoAlign(false, swerve)), (
  new PathPlannerAuto("3--coral")), (
  new PathPlannerAuto("3--3")), (new Cmd_AutoAlign(false, swerve)));
 */
// Rutina 1--1.5
 //return new SequentialCommandGroup(new PathPlannerAuto("1--1"), (new Cmd_AutoAlign(false, swerve)), (new PathPlannerAuto("1--coral station")));


//Rutina 3--3.5
  //return new SequentialCommandGroup(new PathPlannerAuto("3--1"), (new Cmd_AutoAlign(false, swerve)), (new PathPlannerAuto("3--coral station")));
 
 
  }
}
