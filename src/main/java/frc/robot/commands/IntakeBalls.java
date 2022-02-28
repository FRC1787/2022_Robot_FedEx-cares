// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class IntakeBalls extends CommandBase {
  /** Creates a new IntakeBalls. */
  public IntakeBalls(Intake intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Intake.setIntake(Value.kForward); //depends on what the ids are
    Intake.setIntakeMotor(Constants.intakeMotorSpeed); //tweak this value
    Intake.setKowalksiMotor(Constants.kowalskiMotorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.setIntake(Value.kReverse);
    Intake.setIntakeMotor(0);
    Intake.setKowalksiMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
