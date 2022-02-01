// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Camera;

public class LookToTarget extends CommandBase {
  /** Creates a new LookToTarget. */
  public LookToTarget(Drivetrain drivetrain, Camera camera) {
    addRequirements(drivetrain, camera);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Kp = -0.05;
    double minCommand = 0.01;
    double tx = table.getEntry("tx").getDouble(0.0);


    double headingError = -tx;
    double steeringAdjust = 0.0;

    if (tx > 1.0) {
      steeringAdjust = Kp * headingError - minCommand;
    }
    else if (tx < 1.0) {
      steeringAdjust = Kp * headingError + minCommand;
    }
    // dividing by less than 7 causes it to oscillate and doing more than 7 is too slow
    Drivetrain.moveLeftSide(-steeringAdjust/7);
    Drivetrain.moveRightSide(steeringAdjust/7);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
