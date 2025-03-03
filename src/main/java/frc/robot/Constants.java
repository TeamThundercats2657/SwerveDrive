// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
 
    

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {
    
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class AlgaeConstants {
    // PWM ports/CAN IDs for motor controllers
    public static final int kAlgaeSparkID = 12;

    // Current limit for launcher and feed wheels
    public static final int kAlgaeCurrentLimit = 80;

    // Speeds for wheels when intaking and launching for the Algae. Intake speeds are negative to run the wheels
    // in reverse
    public static final double kAlgaeIntakeSpeed = .2;
    public static final double kAlgaeShot = -.5;

    public static final double kAlgaeDelay = 1;

    public static final double kWheelDiameterInches = 4;
}
public static class AlgaeArmConstants {
  // PWM ports/CAN IDs for motor controllers
  public static final int AlgaeArmRMotor = 13;
  public static final int AlgaeArmLMotor = 14;
  // Current limit for launcher and feed wheels
  public static final int kAlgaeDBRCurrentLimit = 80;
  public static final int kAlgaeDBLCurrentLimit = 80;
  // Speeds for wheels when intaking and launching for the Algae. Intake speeds are negative to run the wheels
  // in reverse
  public static final double kAlgaeDBRDown = -.3;
  public static final double kAlgaeDBLUp = -.3;
  public static final double kAlgaeDBRUp = .3;
  public static final double kAlgaeDBLDown = .3;

  

 
}
public static class CoralConstants {
  // PWM ports/CAN IDs for motor controllers
  public static final int kCoralSparkID = 16;

  // Current limit for launcher and feed wheels
  public static final int kCoralCurrentLimit = 80;

  // Speeds for wheels when intaking and launching for the Algae. Intake speeds are negative to run the wheels
  // in reverse
  public static final double kCoralIntakeSpeed = .2;
  public static final double kCoralShot = -.2;

  public static final double kCoralDelay = 1;

  public static final double kWheelDiameterInches = 2;
}
public static class CoralArmConstants {
  // PWM ports/CAN IDs for motor controllers
  public static final int CoralArmMotor = 15;
  // Current limit for launcher and feed wheels
  public static final int kCoralDBRCurrentLimit = 80;
  public static final int kCoralDBLCurrentLimit = 80;
  // Speeds for wheels when intaking and launching for the Algae. Intake speeds are negative to run the wheels
  // in reverse
  public static final double kCoralDown = .4;
  public static final double kCoralUp = .8;

  

 
}
public static class ElevatorConstants {
  // PWM ports/CAN IDs for motor controllers
  public static final int kElevatorSpark = 17;

  // Current limit for launcher and feed wheels
  public static final int kElevatorCurrentLimit = 80;

  // Speeds for wheels when intaking and launching for the Algae. Intake speeds are negative to run the wheels
  // in reverse
  public static final double kElevatorUpSpeed = .4;
  public static final double kElevatorDownSpeed = -.4;

  public static final double kElevatorDelay = 1;

}
public static class AlgaePIDConstants {
  
  
  // Speeds for wheels when intaking and launching for the Algae. Intake speeds are negative to run the wheels
  // in reverse
  public static final double kP = 0.0020645;
  public static final double kI = 0;
  public static final double kD = 0;

  

 
}

public static class CoralPIDConstants {
  
  
  // Speeds for wheels when intaking and launching for the Algae. Intake speeds are negative to run the wheels
  // in reverse
  public static final double kP = 0.0020645;
  public static final double kI = 0;
  public static final double kD = 0;

  

 
}
}