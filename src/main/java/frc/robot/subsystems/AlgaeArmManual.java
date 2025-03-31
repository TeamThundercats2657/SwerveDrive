// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeArmConstants;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArmManual extends SubsystemBase {
  
  private final SparkMax algaeArmRSparkMax;
  private final SparkMax algaeArmLSparkMax;
  private final SparkMaxConfig algaeArmRSparkMaxConfig;
  private final SparkMaxConfig algaeArmLSparkMaxConfig;

  /** Creates a new Amp. */
  public AlgaeArmManual() {
    algaeArmRSparkMax = new SparkMax(AlgaeArmRMotor, SparkMax.MotorType.kBrushless);
    algaeArmLSparkMax = new SparkMax(AlgaeArmLMotor, SparkMax.MotorType.kBrushless);
    algaeArmRSparkMaxConfig = new SparkMaxConfig();
    algaeArmLSparkMaxConfig = new SparkMaxConfig();

    algaeArmRSparkMaxConfig.inverted(true).idleMode(IdleMode.kBrake);
    algaeArmLSparkMaxConfig.inverted(true).idleMode(IdleMode.kBrake);

  
   // m_AmpWheel.setSmartCurrentLimit(kAmpCurrentLimit);
  }

  /**
   * This method is an example of the 'subsystem factory' style of command creation. A method inside
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
          setAlgaeArm(kAlgaeDBRDown);
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
          setAlgaeArm(kAlgaeDBRUp);
        },
        // When the command stops, stop the wheels
        () -> {
          stop();
        });
  }
  // An accessor method to set the speed (technically the output percentage) of the amp intake wheel
  public void setAlgaeArm(double speed) {
    algaeArmRSparkMax.set(speed);
    algaeArmLSparkMax.set(-speed);
  }

  // An accessor method to set the speed (technically the output percentage) of the amp shooter wheel
 

  
  

  // A helper method to stop both wheels. You could skip having a method like this and call the
  // individual accessors with speed = 0 instead
  public void stop() {
    algaeArmRSparkMax.set(0);
    algaeArmLSparkMax.set(0);
  }
  
}

