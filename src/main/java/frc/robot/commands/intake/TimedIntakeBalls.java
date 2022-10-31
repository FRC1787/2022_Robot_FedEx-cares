// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TimedIntakeBalls extends CommandBase {
  Timer timer = new Timer();
  double speed;
  double time;
  
  public TimedIntakeBalls(Intake intake, double speedParam, double timeParam) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    speed=speedParam;
    time=timeParam;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    Intake.setIntakeMotor(-speed);
    Intake.setKowalskiMotor(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop(); timer.reset(); 
    Intake.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
