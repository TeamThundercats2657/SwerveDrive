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

import static frc.robot.Constants.AlgaeArmConstants.*;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class AlgaeArm extends SubsystemBase {
  //private means that the variable can only be accessed within the class
  // final means that the variable can only be assigned once
  private final SparkMax algaeArmRSparkMax = new SparkMax(AlgaeArmRMotor, SparkMax.MotorType.kBrushless);
  private final SparkMax algaeArmLSparkMax = new SparkMax(AlgaeArmLMotor, SparkMax.MotorType.kBrushless);
  private final SparkMaxConfig algaeArmRSparkMaxConfig = new SparkMaxConfig();
  private final SparkMaxConfig algaeArmLSparkMaxConfig = new SparkMaxConfig();
  //setting up the PID controller for the arm
  private final PIDController algaeArmPID = new PIDController(1, 0, 0.2);
  //setting up the encoder for the arm
 private final SparkAbsoluteEncoder algaeArmEncoder  ;
  /** Creates a new AlgaeArm. */
  public AlgaeArm() {
    //created 2 different motors for the arm above on 16 and 17 and we made the configurations on line 18 and 19
    //then we set up the config on lines 24-27 and then applied the config on lines 28 and 29.  
    algaeArmRSparkMaxConfig.inverted(false).idleMode(IdleMode.kBrake);
    algaeArmLSparkMaxConfig.inverted(true).idleMode(IdleMode.kBrake);
    //setting the conversion factor for the encoder
    algaeArmRSparkMaxConfig.encoder.positionConversionFactor((360/12)).velocityConversionFactor((360/12));
    //setting the conversion factor for the encoder
    algaeArmLSparkMaxConfig.encoder.positionConversionFactor((360/12)).velocityConversionFactor((360/12));
    //setting the PID controller for the arm
    algaeArmRSparkMax.configure(algaeArmRSparkMaxConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
   //setting the PID controller for the arm
    algaeArmLSparkMax.configure(algaeArmLSparkMaxConfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
    //setting the encoder for the arm
    algaeArmEncoder = algaeArmRSparkMax.getAbsoluteEncoder();
    algaeArmPID.enableContinuousInput(0,1);
  }
public void setArmSetPoint(double setPointdegrees){
  algaeArmPID.setSetpoint(setPointdegrees/360);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(("Absolute Encoder"), algaeArmEncoder.getPosition());
    algaeArmRSparkMax.set(algaeArmPID.calculate(algaeArmEncoder.getPosition()));
    algaeArmLSparkMax.set(algaeArmPID.calculate(algaeArmEncoder.getPosition()));
    SmartDashboard.putNumber(("Setpoint"), algaeArmPID.getSetpoint());
    SmartDashboard.putNumber(("Output"), algaeArmPID.calculate(algaeArmEncoder.getPosition()));
    SmartDashboard.putData(("PID"), algaeArmPID);
  }
}
