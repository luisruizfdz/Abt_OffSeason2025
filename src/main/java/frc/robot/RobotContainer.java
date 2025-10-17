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
  
    //private final CommandXboxController joysubs = new CommandXboxController(1);
    private final CommandXboxController joydrive = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings

    NamedCommands.registerCommand("Vision derecha", new Cmd_AutoAlign(true, swerve));
    NamedCommands.registerCommand("Vision izquierda", new Cmd_AutoAlign(false, swerve));

    NamedCommands.registerCommand("Intake Coral rollers", new Cmd_IntakeEstrellas(IntakeCoral));
    //NamedCommands.registerCommand("PID Intake", new Cmd_PIDIntake(IntakeCoral, 0));

    NamedCommands.registerCommand("null", getAutonomousCommand());

    swerve.setDefaultCommand(new Cmd_Move_Swerve(swerve, () -> joydrive.getLeftX(), () -> joydrive.getLeftY(), ()-> joydrive.getRightX(),() -> joydrive.rightTrigger().getAsBoolean(),() -> joydrive.y().getAsBoolean()));
  ;


    configureBindings();
    
  }

  
  private void configureBindings() {
    //joydrive.start().whileTrue(new Cmd_zeroheding(swerve));
    
    joydrive.leftBumper().whileTrue(new Cmd_AutoAlign(false, swerve));
    joydrive.rightBumper().whileTrue(new Cmd_AutoAlign(true, swerve));

   // joydrive.x().whileTrue(new Cmd_IndexerRIndependent(Indexer));

   
    //Intake 
    //joydrive.a().whileTrue(new Cmd_PIDIntake(IntakeCoral));

    //joydrive.y().whileTrue(new SequentialCommandGroup(new ParallelCommandGroup(
      //new Cmd_IntakeEstrellas(IntakeCoral), 
      //new Cmd_IndexerRollers(Indexer), 
      //new Cmd_EndEffector(EndEffector)))); 


    //joydrive.x().whileTrue(new Cmd_IntakeEstrellas(IntakeCoral));
    //joydrive.b().whileTrue(new Cmd_IndexerRollers(Indexer));
    //joydrive.a().whileTrue(new Cmd_EndEffector(EndEffector));

    //Elevador
    joydrive.rightTrigger().whileTrue(new Cmd_PIDElevador(SubM));
    joydrive.leftTrigger().whileTrue(new Cmd_PIDElevadorB(SubM));

    //Climber 
    
    joydrive.x().whileTrue(new Cmd_ClimberCable(climber));
    joydrive.y().whileTrue(new Cmd_ClimberCableB(climber));
    joydrive.b().whileTrue(new Cmd_ClimberLlanta(climber)); 

    //CAN Range 

    //joydrive.a().whileTrue(new Cmd_CanRange()); 



    //joydrive.y().whileTrue(new Cmd_CanRange ());
    
   /*   
   joysubs.leftBumper().whileTrue(new SequentialCommandGroup(new ParallelCommandGroup(new Cmd_AlgasIntake_PID(IntakeAlgas, 120),
    new Cmd_RuedasMove (IntakeAlgas))));

    joysubs.rightBumper().whileTrue(new SequentialCommandGroup(new ParallelCommandGroup(new Cmd_AlgasIntake_PID(IntakeAlgas, 0),
    new Cmd_RuedasStop (IntakeAlgas))));
 */

    //Mover el intake de corales
    /*
    joysubs.start().toggleOnTrue(new Cmd_PIDIntake(IntakeCoral, 120));

    
    //Mover los todos los rollers para meter el coral hasta el end effector
    joysubs.rightBumper().whileTrue(new SequentialCommandGroup(new ParallelCommandGroup(
      new Cmd_IndexerRollers(Indexer),
      new Cmd_IntakeEstrellas (IntakeCoral)),
      new Cmd_EndEffector(EndEffector/*falta ver si vamos a usar un sensor )));

      
    //Subir el elevador y activar el end effector para soltar el coral, y luego bajar el elevador
    joysubs.b().toggleOnTrue(new SequentialCommandGroup(
      new Cmd_PIDElevador(Elevador, 9), (
      new Cmd_EndEffector(EndEffector/*falta ver si vamos a usar un sensor )), (
      new Cmd_PIDElevador(Elevador, 0))));


    //Mover todos los rollers al reves en caso de emergencia
    //Falta invertir esos valores
    joysubs.back().whileTrue(new SequentialCommandGroup(new ParallelCommandGroup(
      new Cmd_IndexerRollers(Indexer),
      new Cmd_EndEffector(EndEffector),
      new Cmd_IntakeEstrellas (IntakeCoral))));
*/
      
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
    

//Rutina 1
/* 
return new SequentialCommandGroup (
  new PathPlannerAuto("1--1"), (new Cmd_AutoAlign(false, swerve)), (
  new PathPlannerAuto("1--coral")), (
  new PathPlannerAuto("1--3")), (new Cmd_AutoAlign(false, swerve)));
 */
//Rutina 2

  //return new PathPlannerAuto("2--1");

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
  return new SequentialCommandGroup(new PathPlannerAuto("3--1"), (new Cmd_AutoAlign(false, swerve)), (new PathPlannerAuto("3--coral station")));
 
 
  }
}
