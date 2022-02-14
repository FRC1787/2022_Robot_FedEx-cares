// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetClimbPiston extends InstantCommand {
  /** Creates a new SetClimbPiston. */
  boolean up;
  public SetClimbPiston(Climb climbSubsystem, boolean setPistonUp) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climbSubsystem);
    this.up = setPistonUp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (up) {
      Climb.setPiston(Value.kReverse); //reverse == up position
    }
    else {
      Climb.setPiston(Value.kForward);
    }
  }
}
