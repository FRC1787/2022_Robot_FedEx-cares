// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class TimedIndexBalls extends CommandBase {
  Timer timer = new Timer();
  double time;
  double rpm;

  PIDController indexerPID = new PIDController(Constants.indexerP, Constants.indexerI, Constants.indexerD);
  
  public TimedIndexBalls(Shooter shooter, double timeParam, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    time=timeParam;
    this.rpm = rpm;

    indexerPID.setTolerance(100);
    indexerPID.setSetpoint(Constants.indexerSetpoint);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    Shooter.setIndexerRPM(this.rpm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop(); timer.reset(); 
    Shooter.stopAllMotors();
    Shooter.raiseShooter();
    Intake.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
