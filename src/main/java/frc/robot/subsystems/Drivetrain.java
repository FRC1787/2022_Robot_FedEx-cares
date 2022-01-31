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
   //look into follower motors (one main motor, other ones try and match speed of first one via encoder value)
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

    left1E.setPositionConversionFactor(Constants.positionConversionFactor);
    left2E.setPositionConversionFactor(Constants.positionConversionFactor);
    left3E.setPositionConversionFactor(Constants.positionConversionFactor);
    right1E.setPositionConversionFactor(Constants.positionConversionFactor);
    right2E.setPositionConversionFactor(Constants.positionConversionFactor);
    right3E.setPositionConversionFactor(Constants.positionConversionFactor);

    left1E.setVelocityConversionFactor(Constants.velocityConversionFactor);
    left2E.setVelocityConversionFactor(Constants.velocityConversionFactor);
    left3E.setVelocityConversionFactor(Constants.velocityConversionFactor);
    right1E.setVelocityConversionFactor(Constants.velocityConversionFactor);
    right2E.setVelocityConversionFactor(Constants.velocityConversionFactor);
    right3E.setVelocityConversionFactor(Constants.velocityConversionFactor);

    resetEncoders();
    
    //assumes position of 0, 0
    m_odometry = new DifferentialDriveOdometry(navX.getRotation2d());
  }

  //Resets all the neo encoders to 0
  public static void resetEncoders() {
    left1E.setPosition(0); 
    left2E.setPosition(0);
    left3E.setPosition(0);
    right1E.setPosition(0);
    right2E.setPosition(0);
    right3E.setPosition(0);
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

  //distance traveled in meters by left side
  public static double leftEncoderPosition() {
    //return (left1E.getPosition() + left2E.getPosition() + left3E.getPosition()) / 3.0;
    return left1E.getPosition();
  }

  //distance traveled in meters by right side
  public static double rightEncoderPosition() {
    //return (right1E.getPosition() + right2E.getPosition() + right3E.getPosition()) / 3.0;
    return right1E.getPosition();
  }

  //speed of left side in m/s
  public static double leftEncoderSpeed() {
    //return (left1E.getVelocity() + left2E.getVelocity() + left3E.getVelocity()) / 3.0;
    return left1E.getVelocity();
  }
  
  //speed of right side in m/s
  public static double rightEncoderSpeed() {
    //return (left1E.getVelocity() + left2E.getVelocity() + left3E.getVelocity()) / 3.0;
    return right1E.getVelocity();
  }

  //WE PROBABLY DO NOT NEED ALL OF THIS COMMENTED STUFF
  // //encoder 42 pulses per revolution

  // // distance in feet the right side of the robot has traveled
  // public static double rightDistance() {
  //   // dont know why 2.2 works but it works in place of the gearbox ratio
  //   return ((rightEncoder() / Constants.pulsesPerRotation) / 10.38 * Math.PI * Constants.wheelDiameter);
  // }

  // // distance in feet the left side of the robot has traveled
  // public static double leftDistance() {
  //   return -((leftEncoder() / Constants.pulsesPerRotation) / 2.21 * Math.PI * Constants.wheelDiameter);
  // }

  // //returns speed, idk what the numbers mean (maybe wrong)
  // public double leftDriveSpeed() {
  //   return (left1E.getVelocity() / 60) * Constants.pulsesPerRotation * (0.1524 * Math.PI) / Constants.pulsesPerRotation * Constants.gearboxRatio;
  // }

  // public double rightDriveSpeed() {
  //   return -(right1E.getVelocity() / 60) * Constants.pulsesPerRotation * (0.1524 * Math.PI) / Constants.pulsesPerRotation * Constants.gearboxRatio;
  // }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoderSpeed(), rightEncoderSpeed());
  }

  //returns robot's heading in degrees
  public double getHeading() {
    return navX.getYaw();
  }

  //return the current pose of the robot
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, navX.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update odometry
    m_odometry.update(navX.getRotation2d(), leftEncoderPosition(), rightEncoderPosition());
  
    
  }
}
