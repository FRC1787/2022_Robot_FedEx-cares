// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb;

public class TestClimb extends CommandBase {
  /** Creates a new TestClimb. */
  public TestClimb(Climb climb) {
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double sliderValue = -0.67*RobotContainer.stick.getRawAxis(3);

    if (sliderValue < 0) { //retracts arm
      if (!Climb.isRetracted()) Climb.setArm(sliderValue);
      else Climb.setArm(0);
    }
    else {
      if (!Climb.isExtended()) Climb.setArm(sliderValue);
      else Climb.setArm(0);
    }

    SmartDashboard.putNumber("sliderValue" , sliderValue);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Climb.setArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
