// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorM;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorL1UPwithwhile extends InstantCommand {
  private final ElevatorM m_Elevator;
  
  private double m_start_rotation;
  public ElevatorL1UPwithwhile(ElevatorM Elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = Elevator;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_start_rotation = Math.round(m_Elevator.rotation() * 1000.0) / 1000.0;
    m_Elevator.setElevatorMWheel(kElevatorUpSpeed); 
    long t= System.currentTimeMillis();
    long end = t+3000;
    while(System.currentTimeMillis() < end) {
      // do something
      // pause to avoid churning
     //Thread.sleep( 5000 );
      System.out.println("rotation: " + m_Elevator.rotation());
      double roundedValue = Math.round(m_Elevator.rotation() * 1000.0) / 1000.0;
      if (m_start_rotation == roundedValue) {
        System.out.println(roundedValue + "   STOPPING:   " + m_Elevator.rotation());
        m_Elevator.stop();
   break;
        
   }
  
    
    }
    //try {
    //    Thread.sleep(1000);
    //} catch (InterruptedException e) {
//e.printStackTrace();
    ///}
    m_Elevator.stop();


  }
  
  }

