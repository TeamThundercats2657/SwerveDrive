// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ElevatorConstants.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorM;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorL1Up extends InstantCommand {
  private final ElevatorM m_Elevator;
  public ElevatorL1Up(ElevatorM Elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = Elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
m_Elevator.setElevatorMWheel(kElevatorUpSpeed); 
try {
    Thread.sleep(1000);
} catch (InterruptedException e) {
    e.printStackTrace();
}
m_Elevator.stop();

m_Elevator.setElevatorMWheel(kElevatorUpSpeed); 
try {
    Thread.sleep(1000);
} catch (InterruptedException e) {
    e.printStackTrace();
}
m_Elevator.stop();
  }
  
}
