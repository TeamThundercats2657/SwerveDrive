package frc.robot.commands;

import static frc.robot.Constants.CoralConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralSpinner;


/*This is an exCoralle of creating a command as a class. The base Command class provides a set of methods that your command
 * will override.
 */
public class CoralIntake extends Command {
    CoralSpinner m_Coralintake;
  
    // CANLauncher m_launcher;
    CoralSpinner m_Coralrelease;
  
    /** Creates a new CoralShot. */
    public CoralIntake(CoralSpinner Coralintake) {
      // save the launcher system internally
      m_Coralintake = Coralintake;
      m_Coralrelease = Coralintake;

    // indicate that this command requires the launcher system
    addRequirements(m_Coralintake);
  }

  // The initialize method is called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set the wheels to launching speed
    m_Coralintake.setCoralWheel(kCoralIntakeSpeed);
    m_Coralrelease.setCoralWheel(-kCoralIntakeSpeed);
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
    m_Coralintake.stop();
  }
}


