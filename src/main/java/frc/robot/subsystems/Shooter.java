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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    acceleratorE.setVelocityConversionFactor(1.0);
    backspinnerE.setVelocityConversionFactor(1.0);
    accelerator.setInverted(true);
  }


  /**
   * Sets the speed of the backspinner and accelerator motors
   * 
   * <p>
   * To set the speed of the accelerator motor, input a double between 0 and -1.0. For example,
   * {@code accelerateShooter(-0.5);}
   * </p>
   * <p>
   * To set the speed of the backspinner motor, input a double between 0 and 1.0. For example,
   * {@code accelerateShooter(0.5);}
   * </p>
   * @param acceleratorSpeed - speed of accelerator motor. Value should be between 0 and -1.0
   * @param backspinnerSpeed - speed of backspinner motor. Value should be between 0 and -1.0
   */
  public static void accelerateShooter(double acceleratorSpeed, double backspinnerSpeed) {
    accelerator.set(acceleratorSpeed);
    backspinner.set(backspinnerSpeed);
  }

  /**
   * Sets the position of the shooter's pneumatic cylinder
   * 
   * <p>
   * To raise the shooter, input {@code kForward} for the {@code position} paraneter. For example,
   * {@code setShooterPosition(kForward);}
   * </p>
   * <p>
   * To lower the shooter, input {@code kReverse} for the {@code position} paraneter. For example,
   * {@code setShooterPosition(kReverse);}
   * </p>
   * 
   * @param position - position of the pneimatic cylinder
   */
  public static void setShooterPosition(DoubleSolenoid.Value position) {
    leftPiston.set(position);
  }

  /**
   * Sets the speed of the indexer motor
   * 
   * <p>
   * To set the speed of the motor, input a double between 0 and 1.0. For example,
   * {@code setIndexerSpeed(0.5);}
   * </p>
   * @param speed - Speed to set for the motor. Value should be between 0 and 1.0
   */
  public static void setIndexerSpeed(double speed) {
    indexer.set(speed);
  }

  /**
   * Sets the speed of the accelerator motor
   * 
   * <p>
   * To set the speed of the motor, input a double between 0 and -1.0. For example,
   * {@code setAcceleratorSpeed(-0.5);}
   * </p>
   * @param speed - Speed to set for the motor. Value should be between 0 and -1.0
   */
  public static void setAcceleratorSpeed(double speed) {
    accelerator.set(speed);
  }

  /**
   * Sets the speed of the backspinner motor
   * 
   * <p>
   * To set the speed of the motor, input a double between 0 and 1.0. For example,
   * {@code setBackspinnerSpeed(0.5);}
   * </p>
   * @param speed - Speed to set for the motor. Value should be between 0 and 1.0
   */
  public static void setBackspinnerSpeed(double speed) {
    backspinner.set(speed);
  }

  /**
   * Get the velocity of the backspinner motor. This returns in RPM
   * @return Backspinner motor RPM
   */
  public static double getBackspinnerSpeed() {
    return backspinnerE.getVelocity();
  }

  /**
   * Get the velocity of the accelorator motor. This returns in RPM
   * @return Backspinner motor RPM
   */
  public static double getAcceleratorSpeed() {
    return acceleratorE.getVelocity();
  }

  public static void setAcceleratorRPM(double rpm) {
    accelerator.set(rpm*Constants.acceleratorRPMToPercent);
  }
  
  public static void setBackspinnerRPM(double rpm) {
    backspinner.set(rpm*Constants.backspinnerRPMToPercent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("backspinner speed", getBackspinnerSpeed());
    SmartDashboard.putNumber("accelerator speed", getAcceleratorSpeed());
  }
}