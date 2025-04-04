// This is a command that will run the algae wheels to release the power cells.
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.AlgaeConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSpinner;


/*This is an exAlgaele of creating a command as a class. The base Command class provides a set of methods that your command
 * will override.
 */
public class AlgaeRelease extends Command {
  AlgaeSpinner m_AlgaeRelease;

  // CANReleaseer m_Releaseer;

  /** Creates a new AlgaeShot. */
  public AlgaeRelease(AlgaeSpinner AlgaeRelease) {
    // save the Releaseer system internally
    m_AlgaeRelease = AlgaeRelease;
    

    // indicate that this command requires the Releaseer system
    addRequirements(m_AlgaeRelease);
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the wheels to Releaseing speed
    m_AlgaeRelease.setAlgaeWheel(-kAlgaeIntakeSpeed);
    try {
      wait(1000);
    } catch (InterruptedException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // There is nothing we need this command to do on each iteration. You could remove this method
    // and the default blank method
    // of the base class will run.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Always return false so the command never ends on it's own. In this project we use the
    // scheduler to end the command when the button is released.
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the wheels when the command ends.
    m_AlgaeRelease.stop();
  }
}

   

