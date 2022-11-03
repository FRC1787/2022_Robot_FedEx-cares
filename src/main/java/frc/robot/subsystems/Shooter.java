// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public static boolean isRaised;

  private static CANSparkMax indexer = new CANSparkMax(Constants.indexerMotorID, MotorType.kBrushless);
  private static CANSparkMax flywheel = new CANSparkMax(Constants.flywheelMotorID, MotorType.kBrushless);
  private static CANSparkMax backspinner = new CANSparkMax(Constants.backspinnerMotorID, MotorType.kBrushless);
  
  private static RelativeEncoder flywheelE = flywheel.getEncoder();
  private static RelativeEncoder backspinnerE = backspinner.getEncoder();
  private static RelativeEncoder indexerE = indexer.getEncoder();

  private static DoubleSolenoid piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.shooterPistonForwardID, Constants.shooterPistonReverseID);

  public Shooter() {
    indexer.setIdleMode(IdleMode.kBrake);
    flywheel.setIdleMode(IdleMode.kCoast);
    backspinner.setIdleMode(IdleMode.kCoast);
    flywheel.setOpenLoopRampRate(0);
    backspinner.setOpenLoopRampRate(0);
    flywheelE.setVelocityConversionFactor(1.0);
    backspinnerE.setVelocityConversionFactor(1.0);
    indexerE.setVelocityConversionFactor(1.0);
    flywheel.setInverted(true);
    backspinner.setInverted(true);
    indexer.setInverted(false);
    indexer.setSmartCurrentLimit(90);
    flywheel.setSmartCurrentLimit(90);
    backspinner.setSmartCurrentLimit(90);
    piston.set(DoubleSolenoid.Value.kForward);
    isRaised=true;
  }

  private static NetworkTableEntry flywheelSpeed = Shuffleboard.getTab("AllShot")
    .add("FlywheelRPM", 3600)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 2850, "max", 3600))
    .getEntry();

  private static NetworkTableEntry backspinnerSpeed = Shuffleboard.getTab("AllShot")
    .add("BackspinnerRPM", 3750)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 3000, "max", 3750))
    .getEntry();

  public static double flywheelCustom() {
    return Math.ceil(flywheelSpeed.getDouble(0));
    // return 3400;
  }

  public static double backspinnerCustom() {
    return Math.ceil(backspinnerSpeed.getDouble(0));
    // return 3550;
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
    if (position == DoubleSolenoid.Value.kForward) isRaised = true;
    else isRaised = false;
  }

  /**
   * Toggles the position of the shooter's pneumatic cylinder
   */
  public static void toggleShooterPosition() {
    piston.toggle();
    isRaised = !isRaised;
  
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
   * @return Flywheel motor RPM
   */
  public static double getFlywheelSpeed() {
    return flywheelE.getVelocity();
  }

  public static double getIndexerSpeed() {
    return indexerE.getVelocity();
  }
  /**
   * Sets the speed of the flywheel motor using an RPM value
   * @param rpm - RPM speed to set for the motor
   */
  public static void setFlywheelRPM(double rpm) {
    if(Vision.isOppositeColor()) {
      // rpm = 1000;
    }
    flywheel.set(rpm*0.000196 + 0.00562); //obtained from testing
  }

  /**
   * Sets the speed of the backspinner motor using an RPM value
   * @param rpm - RPM speed to set for the motor
   */
  public static void setBackspinnerRPM(double rpm) {
    if(Vision.isOppositeColor()) {
      // rpm = 1000;
    }
    backspinner.set(rpm*0.000187 - 0.00259);
  }

  public static void setIndexerRPM(double rpm) {
    indexer.set(rpm*0.000182 + 0.0102);
  }
  public static void stopAllMotors() {
    setFlywheelSpeed(0);
    setIndexerSpeed(0);
    setBackspinnerSpeed(0);
  }

  public static void raiseShooter() {
    setShooterPosition(Value.kForward);
  }

  public static void lowerShooter() {
    setShooterPosition(Value.kReverse);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("backspinner speed", getBackspinnerSpeed());
    SmartDashboard.putNumber("flywheel speed", getFlywheelSpeed());
    SmartDashboard.putNumber("indexer speed", getIndexerSpeed());
    SmartDashboard.putNumber("indexer setpoint", Constants.indexerSetpoint);

  }
}