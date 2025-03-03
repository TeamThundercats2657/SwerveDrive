// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeArmDown;
import frc.robot.commands.AlgaeArmUp;
import frc.robot.commands.CoralAllDown;
import frc.robot.commands.CoralAllUp;
import frc.robot.commands.CoralCollectAngle;
import frc.robot.subsystems.ElevatorM;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.AlgaeSpinner;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralSpinner;
import frc.robot.subsystems.CoralArm;
import frc.robot.commands.CoralReleaseAngle;
//import frc.robot.commands.ElevatorL1;
import frc.robot.commands.ElevatorL1;
import frc.robot.commands.ElevatorL2;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
/*The gamepad provided in the KOP shows up like an XBox controller if the mode switch is set to X mode using the
   * switch on the top.*/
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      private final CommandXboxController m_operaterController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
      
      private final AlgaeSpinner m_algaeintake = new AlgaeSpinner();
      private final CoralSpinner m_coralIntake = new CoralSpinner();
      //private final AlgaeDrawbridgeRight m_algaeDBR = new AlgaeDrawbridgeRight();
      //private final AlgaeDrawbridgeLeft m_algaeDBL = new AlgaeDrawbridgeLeft();
      private final AlgaeArm m_algaeArm = new AlgaeArm();
      private final CoralArm m_coralArm = new CoralArm();
      private final ElevatorM m_ElevatorM = new ElevatorM();
      ;
   
  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/falcon"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  double getRightX() {
    return -m_driverController.getRightX();
  }
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * 1,
                                                                () -> m_driverController.getLeftX() * 1)
                                                            .withControllerRotationAxis(this::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(this::getRightX,
                                                                                             m_driverController::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -m_driverController.getLeftY(),
                                                                        () -> -m_driverController.getLeftX())
                                                                    .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  m_driverController.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

 
                                                                               /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    
    // Configure the trigger bindings
    configureBindings(false);
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings(boolean robotOriented)
  {
  
    m_operaterController.leftBumper().whileTrue(m_algaeintake.getAlgaeIntakeCommand());;
    m_operaterController.rightBumper().whileTrue(m_algaeintake.getAlgaeReleaseCommand());;
    m_operaterController.a().onTrue(new AlgaeArmDown(m_algaeArm));
    m_operaterController.b().onTrue(new AlgaeArmUp(m_algaeArm));
    m_operaterController.leftBumper().whileTrue(m_coralIntake.getCoralIntakeCommand());;
    m_operaterController.rightBumper().whileTrue(m_coralIntake.getCoralReleaseCommand());;
    m_operaterController.povLeft().onTrue(new CoralCollectAngle(m_coralArm));
    m_operaterController.povRight().onTrue(new CoralReleaseAngle(m_coralArm));
    m_operaterController.povDown().onTrue(new CoralAllDown(m_coralArm));
    m_operaterController.povUp().onTrue(new CoralAllUp(m_coralArm));
    //m_operaterController.start().onTrue(new ElevatorL1(m_Elevator));
    m_operaterController.x().whileTrue(m_ElevatorM.getElevatorMUpCommand());; 
    m_operaterController.y().whileTrue(m_ElevatorM.getElevatorMDownCommand());; 
    m_operaterController.start().onTrue(new ElevatorL2(m_ElevatorM));
    //m_operaterController.a().whileTrue((m_algaeDBR.getAlgaeDownCommand()));
    //m_operaterController.b().whileTrue((m_algaeDBR.getAlgaeUpCommand()));
    //m_operaterController.a().whileTrue((m_algaeDBL.getAlgaeDownCommand()));
    //m_operaterController.b().whileTrue((m_algaeDBL.getAlgaeUpCommand()));
    //new Trigger(() -> m_operaterController.getRightY() < 0.1).whileTrue(m_algaeDBR.getAlgaeDownCommand());
    //new Trigger(() -> m_operaterController.getRightY() < 0.1).whileTrue(m_algaeDBL.getAlgaeUpCommand());
    
    
    //Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
     //   driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
   // Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
     //   driveDirectAngleKeyboard);

           

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(robotOriented? driveRobotOrientedAngularVelocity:driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      m_driverController.button(7).onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      m_driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(robotOriented? driveRobotOrientedAngularVelocity:driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      m_driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      m_driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.back().whileTrue(drivebase.centerModulesCommand());
      m_driverController.leftBumper().onTrue(Commands.none());
      m_driverController.rightBumper().onTrue(Commands.none());
    } else
    {
      m_driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      m_driverController.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      m_driverController.start().whileTrue(Commands.none());
      m_driverController.back().whileTrue(Commands.none());
      m_driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.rightBumper().onTrue(Commands.none());
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
