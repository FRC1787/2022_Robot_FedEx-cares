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
  private static CANSparkMax flywheel = new CANSparkMax(Constants.flywheelMotorID, MotorType.kBrushless);
  private static CANSparkMax backspinner = new CANSparkMax(Constants.backspinnerMotorID, MotorType.kBrushless);
  
  private static RelativeEncoder flywheelE = flywheel.getEncoder();
  private static RelativeEncoder backspinnerE = backspinner.getEncoder();

  private static DoubleSolenoid piston  = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.shooterPistonForwardID, Constants.shooterPistonReverseID);


  public Shooter() {
    indexer.setIdleMode(IdleMode.kBrake);
    flywheel.setIdleMode(IdleMode.kCoast);
    backspinner.setIdleMode(IdleMode.kCoast);
    flywheel.setOpenLoopRampRate(0);
    backspinner.setOpenLoopRampRate(0);
    flywheelE.setVelocityConversionFactor(1.0);
    backspinnerE.setVelocityConversionFactor(1.0);
    flywheel.setInverted(true);
    piston.set(DoubleSolenoid.Value.kForward);
  }


  /**
   * Sets the speed of the backspinner and flywheel motors
   * 
   * <p>
   * To set the speed of the flywheel motor, input a double between 0 and -1.0. For example,
   * {@code flywheelShooter(-0.5);}
   * </p>
   * <p>
   * To set the speed of the backspinner motor, input a double between 0 and 1.0. For example,
   * {@code flywheelShooter(0.5);}
   * </p>
   * @param flywheelSpeed - speed of flywheel motor. Value should be between 0 and -1.0
   * @param backspinnerSpeed - speed of backspinner motor. Value should be between 0 and -1.0
   */
  public static void flywheelShooter(double flywheelSpeed, double backspinnerSpeed) {
    flywheel.set(flywheelSpeed);
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
    piston.set(position);
  }

  /**
   * Toggles the position of the shooter's pneumatic cylinder
   */
  public static void toggleShooterPosition() {
    piston.toggle();
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
   * Sets the speed of the flywheel motor
   * 
   * <p>
   * To set the speed of the motor, input a double between 0 and -1.0. For example,
   * {@code setFlywheelSpeed(-0.5);}
   * </p>
   * @param speed - Speed to set for the motor. Value should be between 0 and -1.0
   */
  public static void setFlywheelSpeed(double speed) {
    flywheel.set(speed);
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
   * Get the velocity of the flywheel motor. This returns in RPM
   * @return Backspinner motor RPM
   */
  public static double getFlywheelSpeed() {
    return flywheelE.getVelocity();
  }

  /**
   * Sets the speed of the flywheel motor using an RPM value
   * @param rpm - RPM speed to set for the motor
   */
  public static void setFlywheelRPM(double rpm) {
    flywheel.set((rpm)/5150);
  }

  /**
   * Sets the speed of the backspinner motor using an RPM value
   * @param rpm - RPM speed to set for the motor
   */
  public static void setBackspinnerRPM(double rpm) {
    backspinner.set((rpm-100)/5320);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("backspinner speed", getBackspinnerSpeed());
    SmartDashboard.putNumber("flywheel speed", getFlywheelSpeed());
  }
}