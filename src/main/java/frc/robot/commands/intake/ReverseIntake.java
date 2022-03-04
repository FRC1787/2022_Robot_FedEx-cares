// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class ReverseIntake extends CommandBase {
  /** Creates a new ReverseIntake. */
  public ReverseIntake(Intake intakeSubsystem, Shooter shooterSubsystem) {
    addRequirements(intakeSubsystem);
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Shooter.setIndexerSpeed(-0.4);
    Intake.setKowalskiMotor(-0.4);
    Intake.setIntakeMotor(0.7);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.setKowalskiMotor(0);
    Intake.setIntakeMotor(0);
    Shooter.setIndexerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
