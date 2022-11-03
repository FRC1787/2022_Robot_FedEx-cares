// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class WheelToRPM extends CommandBase {
  /** Creates a new ShootBalls. */
  PIDController flywheelPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);
  PIDController backspinnerPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);
  PIDController indexerPID = new PIDController(Constants.indexerP, Constants.indexerI, Constants.indexerD);

  private double flywheelSetpoint;
  private double backspinnerSetpoint;
  
  public WheelToRPM(Shooter shooter, Intake intake) {
    addRequirements(shooter);
    addRequirements(intake);
    
    flywheelPID.setTolerance(100);
    backspinnerPID.setTolerance(100);
    indexerPID.setTolerance(100);

    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    flywheelSetpoint = Vision.calculateFlywheelRPM();
    backspinnerSetpoint = Vision.calculateBackspinnerRPM();

    if(Vision.isOppositeColor()) {
      flywheelSetpoint = 2000;
      backspinnerSetpoint = 1900;
    }

    flywheelPID.setSetpoint(flywheelSetpoint);
    backspinnerPID.setSetpoint(backspinnerSetpoint);
    indexerPID.setSetpoint(Constants.indexerSetpoint);

    Shooter.setFlywheelRPM(
      MathUtil.clamp(flywheelPID.calculate(Shooter.getFlywheelSpeed()), -100, 100)
      + Constants.kfShooter*flywheelSetpoint
    );
    

    Shooter.setBackspinnerRPM(
      MathUtil.clamp(backspinnerPID.calculate(Shooter.getBackspinnerSpeed()), -100, 100)
      + Constants.kfShooter*backspinnerSetpoint
    );

    SmartDashboard.putNumber("indexer pid", indexerPID.calculate(Shooter.getIndexerSpeed()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flywheelPID.atSetpoint() && backspinnerPID.atSetpoint();
  }
}
