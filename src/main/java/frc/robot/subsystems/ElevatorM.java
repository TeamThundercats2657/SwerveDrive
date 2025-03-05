// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ElevatorM extends SubsystemBase {
  SparkMax m_ElevatorMWheel;
  //motor controller
  private final SparkMaxConfig ElevatorSparkMaxConfig = new SparkMaxConfig();

  //setting up the PID controller for the 
  private final PIDController ElevatorPID = new PIDController(1, 0, 0.05);

  //setting up the encoder for the 
 private final SparkAbsoluteEncoder ElevatorEncoder  ;
  /** Creates a new Elevator. */
  public void setElevatorSpeed(double speed) {
    

    // Implementation for setting the elevator speed
    }
    public void rotateTo(int position) {

        // Implementation of the rotateTo method
    }


  /** Creates a new ElevatorM. */
  public ElevatorM() {
    m_ElevatorMWheel = new SparkMax(kElevatorSpark, SparkMax.MotorType.kBrushless);

    //m_ElevatorMWheel.setSmartCurrentLimit(kElevatorMCurrentLimit);
    //created 2 different motors for the  above on 16 and 17 and we made the configurations on line 18 and 19
    //then we set up the config on lines 24-27 and then applied the config on lines 28 and 29.  
    ElevatorSparkMaxConfig.inverted(false).idleMode(IdleMode.kBrake);
    //setting the conversion factor for the encoder
    ElevatorSparkMaxConfig.encoder.positionConversionFactor((360/12)).velocityConversionFactor((360/12));
    //setting the conversion factor for the encoder
    //setting the PID controller for the 
    m_ElevatorMWheel.configure(ElevatorSparkMaxConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
   //setting the PID controller for the //setting the encoder for the 
   ElevatorEncoder = m_ElevatorMWheel.getAbsoluteEncoder();
    ElevatorPID.enableContinuousInput(0,1);
    ElevatorPID.setSetpoint(ElevatorEncoder.getPosition()); 
  }
  
  
public void setSetPoint(double setPointrotations){
  ElevatorPID.setSetpoint(setPointrotations/360);
  }
@Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(("ElevatorAbsolute Encoder"), ElevatorEncoder.getPosition());
    //m_ElevatorMWheel.set(ElevatorPID.calculate(ElevatorEncoder.getPosition()));
    SmartDashboard.putNumber(("ElevatorSetpoint"), ElevatorPID.getSetpoint());
    SmartDashboard.putNumber(("ElevatorOutput"), ElevatorPID.calculate(ElevatorEncoder.getPosition()));
    SmartDashboard.putData(("ElevatorPID"), ElevatorPID);//this is the PID controller
  }

  /**
   * This method is an exElevatorMle of the 'subsystem factory' style of command creation. A method inside
   * the subsytem is created to return an instance of a command. This works for commands that
   * operate on only that subsystem, a similar approach can be done in RobotContainer for commands
   * that need to span subsystems. The Subsystem class has helper methods, such as the startEnd
   * method used here, to create these commands.
   */
  public Command getElevatorMUpCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setElevatorMWheel(kElevatorDownSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }
  public Command getElevatorMDownCommand() {
    // The startEnd helper method takes a method to call when the command is initialized and one to
    // call when it ends
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          setElevatorMWheel(kElevatorUpSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }

  // An accessor method to set the speed (technically the output percentage) of the ElevatorM intake wheel
  public void setElevatorMWheel(double speed) {
    m_ElevatorMWheel.set(speed);
  }
  public void setElevatorMLaunch(double speed) {
    m_ElevatorMWheel.set(speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the ElevatorM shooter wheel
 

  
  

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    m_ElevatorMWheel.set(0);
  }
  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public double rotation() {
    return ElevatorPID.calculate(ElevatorEncoder.getPosition());
    
  }
}


        

