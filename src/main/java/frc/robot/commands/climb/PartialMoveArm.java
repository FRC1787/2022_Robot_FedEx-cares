// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class PartialMoveArm extends CommandBase {
  /** Creates a new PartialMoveArm. */
  Timer timer = new Timer();
  double speed;
  double time;
  
  public PartialMoveArm(Climb climbSubsystem, double speed, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSubsystem);
    this.speed=speed;
    this.time=time;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    Climb.setArm(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climb.setArm(0);
    timer.stop(); timer.reset(); //idk if these two are required
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (timer.get() > time || Climb.isExtended());
  }
}
