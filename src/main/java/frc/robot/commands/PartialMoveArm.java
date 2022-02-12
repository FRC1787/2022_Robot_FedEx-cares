// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class PartialMoveArm extends CommandBase {
  /** Creates a new PartialMoveCommand. */

  double speed, time;
  //if we keep this for the final design make this so you can control the distance travelled
  public PartialMoveArm(Climb climbSubsystem, double speed, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSubsystem);
    this.speed = speed;
    this.time = time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Climb.partialMoveArm();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
