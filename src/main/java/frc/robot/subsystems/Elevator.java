// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static frc.robot.Constants.ElevatorConstants.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Elevator extends SubsystemBase {
  //private means that the variable can only be accessed within the class
  // final means that the variable can only be assigned once
  private final SparkMax ElevatorSparkMax = new SparkMax(kElevatorSpark, SparkMax.MotorType.kBrushless);
  private final SparkMaxConfig ElevatorSparkMaxConfig = new SparkMaxConfig();
  //setting up the PID controller for the arm
  private final PIDController ElevatorPID = new PIDController(.25, 0, 0.05);
  //setting up the encoder for the arm
 private final SparkAbsoluteEncoder ElevatorEncoder  ;
  /** Creates a new Elevator. */
  public Elevator() {
    //created 2 different motors for the arm above on 16 and 17 and we made the configurations on line 18 and 19
    //then we set up the config on lines 24-27 and then applied the config on lines 28 and 29.  
    ElevatorSparkMaxConfig.inverted(false).idleMode(IdleMode.kBrake);
    //setting the conversion factor for the encoder
    ElevatorSparkMaxConfig.encoder.positionConversionFactor((360/12)).velocityConversionFactor((360/12));
    //setting the conversion factor for the encoder
    //setting the PID controller for the arm
    ElevatorSparkMax.configure(ElevatorSparkMaxConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
   //setting the PID controller for the arm//setting the encoder for the arm
    ElevatorEncoder = ElevatorSparkMax.getAbsoluteEncoder();
    ElevatorPID.enableContinuousInput(0,1);
    ElevatorPID.setSetpoint(ElevatorEncoder.getPosition()); 
  }
  
  
public void setArmSetPoint(double setPointdegrees){
  ElevatorPID.setSetpoint(setPointdegrees/360);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(("Elevator Absolute Encoder"), ElevatorEncoder.getPosition());
    ElevatorSparkMax.set(ElevatorPID.calculate(ElevatorEncoder.getPosition()));
    SmartDashboard.putNumber(("elvator Setpoint"), ElevatorPID.getSetpoint());
    SmartDashboard.putNumber(("Elvator Output"), ElevatorPID.calculate(ElevatorEncoder.getPosition()));
    SmartDashboard.putData(("ElevatorPID"), ElevatorPID);//this is the PID controller
  }
}


