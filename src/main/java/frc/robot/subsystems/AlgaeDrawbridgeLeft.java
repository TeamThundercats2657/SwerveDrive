package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeDrawbridgeConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeDrawbridgeLeft extends SubsystemBase {
  SparkMax algaeDrawbridgeSparkMax;
  


  /** Creates a new Algae. */
  public AlgaeDrawbridgeLeft() {
    algaeDrawbridgeSparkMax = new SparkMax(AlgaeDBRLMotor    , MotorType.kBrushless);

    //m_AlgaeWheel.setSmartCurrentLimit(kAlgaeCurrentLimit);
  }

  /**
   * This method is an exAlgaele of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands.
   */
  public Command getAlgaeDownCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setAlgaeDown(kAlgaeDBLDown);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }
  public Command getAlgaeUpCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setAlgaeUp(kAlgaeDBLUp);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the Algae intake wheel
  public void setAlgaeDown(double speed) {
    algaeDrawbridgeSparkMax.set(speed);
  }
  public void setAlgaeUp(double speed) {
    algaeDrawbridgeSparkMax.set(speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the Algae shooter wheel
 

  
  

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    algaeDrawbridgeSparkMax.set(0);
  }
  
}

   




