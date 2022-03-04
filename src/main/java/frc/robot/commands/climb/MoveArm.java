// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class MoveArm extends CommandBase {
  /** Creates a new moveArm. */
  double speed;


  /**
   * 
   * Fully extends or rectracts climb arm at provided speed
   * 
   * @param speed - percentage of motor output: positive if extending, negative if retracting
   * 
   */
  public MoveArm(Climb climbSubsystem, double speedParam) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSubsystem);
    speed = speedParam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Climb.setArm(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climb.setArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (speed < 0) return Climb.isRetracted();
    else if (speed > 0) return Climb.isExtended();
    return false;
  }
}
