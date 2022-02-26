// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private static CANSparkMax indexer     = new CANSparkMax(Constants.indexerMotorID, MotorType.kBrushless);
  private static CANSparkMax accelerator = new CANSparkMax(Constants.acceleratorMotorID, MotorType.kBrushless);
  private static CANSparkMax backspinner = new CANSparkMax(Constants.backspinnerMotorID, MotorType.kBrushless);
  
  private static RelativeEncoder acceleratorE = accelerator.getEncoder();
  private static RelativeEncoder backspinnerE = backspinner.getEncoder();

  private static DoubleSolenoid leftPiston  = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.shooterPistonForwardID, Constants.shooterPistonReverseID);


  public Shooter() {
    indexer.setIdleMode(IdleMode.kBrake);
    accelerator.setIdleMode(IdleMode.kCoast);
    backspinner.setIdleMode(IdleMode.kCoast);
  }

  public static void accelerateShooter(double acceleratorSpeed, double backspinnerSpeed) {
    accelerator.set(acceleratorSpeed);
    backspinner.set(backspinnerSpeed);
  }

  public static void setShooterPosition(DoubleSolenoid.Value position) {
    leftPiston.set(position);
  }

  public static void setIndexerSpeed(double speed) {
    indexer.set(speed);
  }

  public static void setAcceleratorSpeed(double speed) {
    accelerator.set(speed);
  }

  public static void setBackspinnerSpeed(double speed) {
    backspinner.set(speed);
  }

  public static double getBackspinnerSpeed() {
    return backspinnerE.getVelocity();
  }

  public static double getAcceleratorSpeed() {
    return acceleratorE.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}