// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Drivetrain extends SubsystemBase {
   /*Spark Max Motor Controller Objects*/
   private static CANSparkMax left1 = new CANSparkMax(20, MotorType.kBrushless);
   private static CANSparkMax left2 = new CANSparkMax(1, MotorType.kBrushless);
   private static CANSparkMax left3 = new CANSparkMax(2, MotorType.kBrushless);
   private static CANSparkMax right1 = new CANSparkMax(13, MotorType.kBrushless);
   private static CANSparkMax right2 = new CANSparkMax(14, MotorType.kBrushless);
   private static CANSparkMax right3 = new CANSparkMax(15, MotorType.kBrushless);
 
   /*Neo Motor Encoder Objects*/
   public static RelativeEncoder left1E = left1.getEncoder();
   public static RelativeEncoder left2E = left2.getEncoder();
   public static RelativeEncoder left3E = left3.getEncoder();
   public static RelativeEncoder right1E = right1.getEncoder();
   public static RelativeEncoder right2E = right2.getEncoder();
   public static RelativeEncoder right3E = right3.getEncoder();

   public static AHRS navX = new AHRS();

  
  private final DifferentialDriveOdometry m_odometry;


  public Drivetrain() {
    left1.setIdleMode(IdleMode.kBrake); //sets drive motors to brake mode
    right1.setIdleMode(IdleMode.kBrake);
    left2.setIdleMode(IdleMode.kBrake);
    right2.setIdleMode(IdleMode.kBrake);
    left3.setIdleMode(IdleMode.kBrake);
    right3.setIdleMode(IdleMode.kBrake);

    right1.setInverted(true); 
    right2.setInverted(true);
    right3.setInverted(true);
    left1.setInverted(false);
    left2.setInverted(false);
    left3.setInverted(false);

    left1E.setPosition(0); //Resets all the neo encoders to 0
    left2E.setPosition(0);
    left3E.setPosition(0);
    right2E.setPosition(0);
    right2E.setPosition(0);
    right3E.setPosition(0);

    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d());
  }

  public static void moveLeftSide(final double speed) {
    left1.set(speed);
    left2.set(speed);
    left3.set(speed);
  }

  public static void moveRightSide(final double speed) {
    right1.set(speed);
    right2.set(speed);
    right3.set(speed);
  }

  //moves left and right sides with voltage
  public void tankDrive(double leftVolts, double rightVolts) {
    left1.setVoltage(leftVolts);
    left2.setVoltage(leftVolts);
    left3.setVoltage(leftVolts);
    right1.setVoltage(rightVolts);
    right2.setVoltage(rightVolts);
    right3.setVoltage(rightVolts);
  }

  // average encoder value on the left side of the robot
  public static double leftEncoder() {
    return -(left1E.getPosition() + left2E.getPosition() + left3E.getPosition()) / 3.0;
  }

  // average encoder value on the right side of the robot
  public static double rightEncoder() {
    return (right1E.getPosition() + right2E.getPosition() + right3E.getPosition()) / 3.0;
  }

  // distance in feet the right side of the robot has traveled
  public static double rightDistance() {
    return ((rightEncoder() / 8.05) * 4 * Math.PI);
  }

  // distance in feet the left side of the robot has traveled
  public static double leftDistance() {
    return -((leftEncoder() / 8.05) * 4 * Math.PI);
  }

  //returns speed, idk what the numbers mean (maybe wrong)
  public double leftDriveSpeed() {
    return (left1E.getVelocity() / 60) * 42 * (0.1524 * Math.PI) / 42 * 10.38;
  }

  public double rightDriveSpeed() {
    return -(right1E.getVelocity() / 60) * 42 * (0.1524 * Math.PI) / 42 * 10.38;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftDriveSpeed(), rightDriveSpeed());
  }

  //returns robot's heading in degrees
  public double getHeading() {
    return navX.getYaw();
  }

  //return the current pose of the robot
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }



  @Override
  public void periodic() {
    // Update odometry
    m_odometry.update(navX.getRotation2d(), leftDistance(), rightDistance());
  
    
  }
}
