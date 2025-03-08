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
public class ElevatorShane extends InstantCommand {
  private final ElevatorM m_Elevator;
  
  private double m_start_rotation;
  public ElevatorShane(ElevatorM Elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = Elevator;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //in your Initialize method, maybe add this:


// Set the wheels to launching speed
// m_ElevatorMWheel.setElevatorMWheel(kElevatorUpSpeed);
// m_ElevatorMWheel.rotation();

double a = 123.13698;
double roundValue = Math.round(a*1000)/1000; // Should be the value you have


int desiredLevel = 2; //What is the desired level, when done, have the button pushed send which level is desired
int currentLevel = m_Elevator.getLevel(); //Get the Elevators current level
int differenceLevel = desiredLevel - currentLevel; //Get the difference between the 2

try
{
  if(differenceLevel != 0 ){
  //Set the direction of travel
    if(differenceLevel > 0 ){
      m_Elevator.setElevatorMWheel(kElevatorUpSpeed); // Only go up
      //could just use power, just make sure it is positive
    } else {
      m_Elevator.setElevatorMWheel(kElevatorDownSpeed); // Only go down since the difference is a negative number
      //could use power, just make sure it is negative
    }

    int i = 0;
    //Loop through each level to change to
    while (i <= differenceLevel) {
      //set the inner loop max or timeout
      long t= System.currentTimeMillis();
      long end = t+1000;
      while(System.currentTimeMillis() < end) {
        long rotationRounded = Math.round(m_Elevator.rotation()*1000)/1000;
        System.out.println("Rounded Value" + roundValue + ", rotatedRoundedValue:" + rotationRounded);
        if(roundValue == rotationRounded){
          System.out.println("----SHOULD BREAK-----");
          break;
        }
      }

      System.out.println("----DONE WITH LEVEL-----");
      System.out.println(i);
      i++;
    }

  //Now is the time to break
  System.out.println("----ALL DONE-----");
  m_Elevator.stop();
  }
} catch (Exception e)
{
    m_Elevator.stop();
    throw new RuntimeException(e);
}







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

