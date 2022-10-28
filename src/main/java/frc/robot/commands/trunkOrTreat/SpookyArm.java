// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trunkOrTreat;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climb;

public class SpookyArm extends CommandBase {

  Climb climber;
  double speed;
  public SpookyArm(Climb climber, double speed) {
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(Climb.isExtended()) {
      Climb.setArm(-speed);
    }
    else if (Climb.isRetracted()) {
      Climb.setArm(speed);
    }

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
