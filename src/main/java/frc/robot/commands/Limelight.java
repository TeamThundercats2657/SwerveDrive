package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.*;;

public class Limelight extends Command {
boolean m_done=false;
SwerveSubsystem  mdrivetrain; 
   /** Creates a new LaunchNote. */
  public Limelight(SwerveSubsystem drivetrain) {
    // save the launcher system internally
    mdrivetrain = drivetrain;}
// The initialize method is called when the command is initially scheduled.
@Override
public void initialize() {
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
  // There is nothing we need this command to do on each iteration. You could remove this method
  // and the default blank method
  // of the base class will run.
  // get the default instance of NetworkTables
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // get the subtable called "datatable"
    NetworkTable table = inst.getTable("limelight");
double targetOffsetAngle_Horizontal = table.getValue("tx").getDouble();
double targetOffsetAngle_Vertical = table.getValue("ty").getDouble();
double tolerance = 1.25;
if (targetOffsetAngle_Vertical < -tolerance) {
        
}
else if (targetOffsetAngle_Vertical > tolerance){
    
 
}
else if (targetOffsetAngle_Horizontal < -tolerance) {
    
}
else if (targetOffsetAngle_Horizontal > tolerance) {
  
}

}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  // Always return false so the command never ends on it's own. In this project we use the
  // scheduler to end the command when the button is released.
  return m_done;
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
}
}



