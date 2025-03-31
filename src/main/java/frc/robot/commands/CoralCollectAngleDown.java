package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralCollectAngleDown extends InstantCommand {
  private final CoralArm m_CoralArm;
  public CoralCollectAngleDown(CoralArm CoralArm) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_CoralArm = CoralArm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
m_CoralArm.setArmSetPoint(240  );
  }
}
    

