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

import static frc.robot.Constants.CoralArmConstants.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class CoralArm extends SubsystemBase {
  //private means that the variable can only be accessed within the class
  // final means that the variable can only be assigned once
  private final SparkMax CoralArmSparkMax = new SparkMax(CoralArmMotor, SparkMax.MotorType.kBrushless);
  private final SparkMaxConfig CoralArmSparkMaxConfig = new SparkMaxConfig();
  //setting up the PID controller for the arm
  private final PIDController CoralArmPID = new PIDController(.8, 0, 0.05);
  //setting up the encoder for the arm
 private final SparkAbsoluteEncoder CoralArmEncoder  ;
  /** Creates a new CoralArm. */
  public CoralArm() {
    //created 2 different motors for the arm above on 16 and 17 and we made the configurations on line 18 and 19
    //then we set up the config on lines 24-27 and then applied the config on lines 28 and 29.  
    CoralArmSparkMaxConfig.inverted(false).idleMode(IdleMode.kBrake);
    //setting the conversion factor for the encoder
    CoralArmSparkMaxConfig.encoder.positionConversionFactor((360/12)).velocityConversionFactor((360/12));
    //setting the conversion factor for the encoder
    //setting the PID controller for the arm
    CoralArmSparkMax.configure(CoralArmSparkMaxConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
   //setting the PID controller for the arm//setting the encoder for the arm
    CoralArmEncoder = CoralArmSparkMax.getAbsoluteEncoder();
    CoralArmPID.enableContinuousInput(0,1);
    CoralArmPID.setSetpoint(CoralArmEncoder.getPosition()); 
  }
  
  
public void setArmSetPoint(double setPointdegrees){
  CoralArmPID.setSetpoint(setPointdegrees/360);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(("CoralAbsolute Encoder"), CoralArmEncoder.getPosition());
    CoralArmSparkMax.set(CoralArmPID.calculate(CoralArmEncoder.getPosition()));
    SmartDashboard.putNumber(("CoralSetpoint"), CoralArmPID.getSetpoint());
    SmartDashboard.putNumber(("CoralOutput"), CoralArmPID.calculate(CoralArmEncoder.getPosition()));
    SmartDashboard.putData(("CoralArmPID"), CoralArmPID);//this is the PID controller
  }
}

