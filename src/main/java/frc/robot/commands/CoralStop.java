// This is a command that will run the Coral wheels to release the power cells.
 // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.CoralConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.CoralSpinner;


/*This is an exCoralle of creating a command as a class. The base Command class provides a set of methods that your command
 * will override.
 */
public class CoralStop extends Command {
  CoralSpinner m_CoralRelease;

  // CANReleaseer m_Releaseer;

  /** Creates a new CoralShot. */
  public CoralStop(CoralArm m_coralArm) {
    // save the Releaseer system internally
    m_CoralRelease = m_CoralRelease;
    

    // indicate that this command requires the Releaseer system
    addRequirements(m_CoralRelease);
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the wheels to Releaseing speed
    m_CoralRelease.setCoralWheel(0);
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
    m_CoralRelease.stop();
  }
}

   

