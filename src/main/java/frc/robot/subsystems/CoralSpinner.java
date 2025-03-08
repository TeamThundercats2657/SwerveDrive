package frc.robot.subsystems;

import static frc.robot.Constants.CoralConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSpinner extends SubsystemBase {
  SparkMax m_CoralWheel;
  


  /** Creates a new Coral. */
  public CoralSpinner() {
    m_CoralWheel = new SparkMax(kCoralSparkID, MotorType.kBrushless);

    //m_CoralWheel.setSmartCurrentLimit(kCoralCurrentLimit);
  }

  /**
   * This method is an exCoralle of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands.
   */
  public Command getCoralIntakeCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setCoralWheel(kCoralIntakeSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }
  public Command getCoralReleaseCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setCoralWheel(-kCoralIntakeSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the Coral intake wheel
  public void setCoralWheel(double speed) {
    m_CoralWheel.set(speed);
  }
  public void setCoralRelease(double speed) {
    m_CoralWheel.set(speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the Coral shooter wheel
 

  
  

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_CoralWheel.set(0);
  }
  
}

   


