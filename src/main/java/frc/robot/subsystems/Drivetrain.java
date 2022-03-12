// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import frc.robot.Robot;

public class Drivetrain extends SubsystemBase {
   /*Spark Max Motor Controller Objects*/
   //look into follower motors (one main motor, other ones try and match speed of first one via encoder value)
   private static CANSparkMax left1 = new CANSparkMax(Constants.l1MotorID, MotorType.kBrushless);
   private static CANSparkMax left2 = new CANSparkMax(Constants.l2MotorID, MotorType.kBrushless);
   private static CANSparkMax right1 = new CANSparkMax(Constants.r1MotorID, MotorType.kBrushless);
   private static CANSparkMax right2 = new CANSparkMax(Constants.r2MotorID, MotorType.kBrushless);
 
   /*Neo Motor Encoder Objects*/
   private static RelativeEncoder left1E = left1.getEncoder();
   private static RelativeEncoder left2E = left2.getEncoder();
   private static RelativeEncoder right1E = right1.getEncoder();
   private static RelativeEncoder right2E = right2.getEncoder();

   private static AHRS gyro = new AHRS();

   private static final Spark blinkin = new Spark(9);
  
  private final DifferentialDriveOdometry odometry;

  public Drivetrain() {
    left1.setIdleMode(IdleMode.kBrake); //sets drive motors to brake mode
    right1.setIdleMode(IdleMode.kBrake);
    left2.setIdleMode(IdleMode.kBrake);
    right2.setIdleMode(IdleMode.kBrake);

    right1.setInverted(false); 
    right2.setInverted(false);
    left1.setInverted(true);
    left2.setInverted(true);

    left1E.setPositionConversionFactor(Constants.positionConversionFactor);
    left1E.setVelocityConversionFactor(Constants.velocityConversionFactor);
    left2E.setPositionConversionFactor(Constants.positionConversionFactor);
    left2E.setVelocityConversionFactor(Constants.velocityConversionFactor);
    right1E.setPositionConversionFactor(Constants.positionConversionFactor);
    right1E.setVelocityConversionFactor(Constants.velocityConversionFactor);
    right2E.setPositionConversionFactor(Constants.positionConversionFactor);
    right2E.setVelocityConversionFactor(Constants.velocityConversionFactor);
    
    resetEncoders();

    //assumes position of 0, 0
    odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    
    // Sets the ramp rate of the drivetrain motors to 1 second

    setAmpLimit(50);
  }

  /**
   * Sets the ramp rate for NEO motors
   *
   * <p>
   * This is the maximum rate at which the motor controller's output is allowed to change.
   * </p>
   * @param rate Time in seconds to go from 0 to full throttle.
   */
  public static void setRampRate(double rate) {
    left1.setOpenLoopRampRate(rate);
    left2.setOpenLoopRampRate(rate);
    right1.setOpenLoopRampRate(rate);
    right2.setOpenLoopRampRate(rate);
  }

  /**
   * Sets the maximum ampere limit for the NEO motors in the drivetrain
   * 
   * @param limit - limit in amps
   */
  public static void setAmpLimit(int limit) {
    left1.setSmartCurrentLimit(limit);
    left2.setSmartCurrentLimit(limit);
    right1.setSmartCurrentLimit(limit);
    right2.setSmartCurrentLimit(limit);
  }

  /**
   * Resets the position of all NEO encoders to 0
   */
  public static void resetEncoders() {
    left1E.setPosition(0); 
    left2E.setPosition(0);
    right1E.setPosition(0);
    right2E.setPosition(0);
  }
  /**
   * Sets the NEO motors of the left side of the robot to the specified speed
   * 
   * <p>
   * To drive the left side foward, input a positive speed. For example,
   * {@code moveLeftSide(0.5);}
   * </p>
   * <p>
   * To drive the left side backward, input a negative speed. For example,
   * {@code moveLeftSide(-0.5);}
   * </p>
   * 
   * @param speed - the speed to set the motors to. Value should be between -1.0 and 1.0
   * 
   */
  public static void moveLeftSide(final double speed) {
    left1.set(speed);
    left2.set(speed);
  }

  /**
   * Sets the NEO motors of the right side of the robot to the specified speed
   * 
   * <p>
   * To drive the right side foward, input a positive speed. For example,
   * {@code moveRightSide(0.5);}
   * </p>
   * <p>
   * To drive the right side backward, input a negative speed. For example,
   * {@code moveRightSide(-0.5);}
   * </p>
   * 
   * @param speed - the speed to set the motors to. Value should be between -1.0 and 1.0
   * 
   */
  public static void moveRightSide(final double speed) {
    right1.set(speed);
    right2.set(speed);
  }

  //moves left and right sides with voltage

  /**
   * Sets the speed of the NEO motors on the left side of the robot to the specified voltage
   * 
   * @param leftVolts - the voltage to set the NEO motors on the left side to. 
   * @param rightVolts
   */
  public static void tankDrive(double leftVolts, double rightVolts) {
    left1.setVoltage(leftVolts);
    left2.setVoltage(leftVolts);
    right1.setVoltage(rightVolts);
    right2.setVoltage(rightVolts);
  }

  /**
   * @return Distance traveled in meters by the left side of the drivetrain
   */
  public static double leftEncoderPosition() {
    return (left1E.getPosition()+left2E.getPosition())/2.0;
  }

  /**
   * @return Distance traveled in meters by the right side of the drivetrain
   */
  public static double rightEncoderPosition() {
    return (right1E.getPosition()+right1E.getPosition())/2.0;
  }

  /**
   * @return RPM of the left side of the drivetrain
   */
  public static double leftEncoderSpeed() {
    return (left1E.getVelocity()+left2E.getVelocity())/2.0;
  }
  
  /**
  * @return RPM of the right side of the drivetrain
  */
  public static double rightEncoderSpeed() {
    return (right1E.getVelocity()+right2E.getVelocity())/2.0;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoderSpeed(), rightEncoderSpeed());
  }

  /**
  * Returns the robot's current yaw value (in degrees, from -180 to 180)
  * reported by the sensor. 
  *
  * @return The robot's current yaw value in degrees (-180 to 180).
  */
  public static double getHeading() {
    return gyro.getYaw();
  }

  /**
   * Return's the robot's current position
   * @return The pose of the robot in meters
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the position of the robot
   * @param pose - the position on the field the robot is at
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  /**
   * Resets the gyro's Z (Yaw) axis to a heading of 0
   */
  public static void resetGyro() {
    gyro.reset();
    
  }

  public static void calibrateGyro() {
    gyro.calibrate();
  }

  public static void setBlinkin(double value) {
    blinkin.set(value);
  }

  @Override
  public void periodic() {
    // Update odometry
    odometry.update(gyro.getRotation2d(), leftEncoderPosition(), rightEncoderPosition());
    SmartDashboard.putNumber("gyro", getHeading());
    setBlinkin(0.81-Math.abs(leftEncoderSpeed()+rightEncoderSpeed()/30));
    SmartDashboard.putBoolean("inAuto", Robot.inAuto);
  }
}
