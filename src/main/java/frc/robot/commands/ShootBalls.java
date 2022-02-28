// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Camera;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ShootBalls extends CommandBase {
  /** Creates a new ShootBalls. */
  PIDController acceleratorPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);
  PIDController backspinnerPID = new PIDController(Constants.kpShooter, Constants.kiShooter, Constants.kdShooter);

  private double acceleratorSetpoint;
  private double backspinnerSetpoint;

  public ShootBalls(Shooter shootSubsystem, Camera cameraSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shootSubsystem);
    addRequirements(cameraSubsystem);
    acceleratorPID.setTolerance(10);
    backspinnerPID.setTolerance(10);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    acceleratorSetpoint = Camera.calculateAcceleratorSpeed(); //function will change depending on threshold
    backspinnerSetpoint = Camera.calculateBackspinnerSpeed();
    if (Camera.distToTarget < Constants.shooterDistanceThreshold) {
      Shooter.setShooterPosition(DoubleSolenoid.Value.kForward);
    }
    else Shooter.setShooterPosition(DoubleSolenoid.Value.kReverse);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Shooter.setAcceleratorSpeed(
      acceleratorPID.calculate(
        Shooter.getAcceleratorSpeed(), acceleratorSetpoint
      )*Constants.shooterRPMToPercent
    );

    Shooter.setBackspinnerSpeed(
      backspinnerPID.calculate(
        Shooter.getBackspinnerSpeed(), backspinnerSetpoint
      )*Constants.shooterRPMToPercent //we are using the same conversion factors for both motors because i dont care
    );


    if (acceleratorPID.atSetpoint() && backspinnerPID.atSetpoint()) {
      Shooter.setIndexerSpeed(0.2);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Shooter.setAcceleratorSpeed(0);
    Shooter.setIndexerSpeed(0);
    Shooter.setBackspinnerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
