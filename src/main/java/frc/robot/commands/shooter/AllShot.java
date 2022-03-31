// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AllShot extends CommandBase {
  /** Creates a new AllShot. */

  PIDController flywheelPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);
  PIDController backspinnerPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);

  private double flywheelSetpoint;
  private double backspinnerSetpoint;
  
  public AllShot(Shooter shooter, Intake intake) {
    addRequirements(shooter);
    addRequirements(intake);
    
    flywheelPID.setTolerance(100);
    backspinnerPID.setTolerance(100);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheelSetpoint = Vision.flywheelAllShotRPM(); 
    backspinnerSetpoint = Vision.backspinnerAllShotRPM();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putNumber("flywheel lower bound", flywheelSetpoint-100);
    SmartDashboard.putNumber("flywheel upper bound", flywheelSetpoint+100);
    SmartDashboard.putNumber("flywheel setpoint", flywheelSetpoint);

    Shooter.setFlywheelRPM(
      MathUtil.clamp(flywheelPID.calculate(Shooter.getFlywheelSpeed(), flywheelSetpoint), -75, 75)
      + Constants.kfShooter*flywheelSetpoint
    );
    

    Shooter.setBackspinnerRPM(
      MathUtil.clamp(backspinnerPID.calculate(Shooter.getBackspinnerSpeed(), backspinnerSetpoint), -75, 75)
      + Constants.kfShooter*backspinnerSetpoint
    );


    if (flywheelPID.atSetpoint() && backspinnerPID.atSetpoint()) {
      Shooter.setIndexerSpeed(0.4);
      Intake.setKowalskiMotor(0.6);
      Intake.setIntakeMotor(-0.6);
    }
    else {
      Shooter.setIndexerSpeed(0);
      Intake.stopAllMotors();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.stopAllMotors();
    Intake.stopAllMotors();
    Shooter.raiseShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return (Shooter.isRaised && Vision.getLimelightA() == 0);
    return false;
  }
}