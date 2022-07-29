// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  // MEMES
  public static final boolean botMostLikelyToGetPHD        = true;
  public static final boolean botWithWorstPathweaverSystem = true;
  public static final boolean FedEx_cares                  = true; // can confirm tho
  public static final boolean bestSchoolTeamAtOrange       = true;
  public static final boolean floppyDiskIsACoolRobot       = true;


  // CAMERA
    public static int cameraWidth = 120;
    public static int cameraHeight = 60;
    public static int cameraFPS = 18;

  //AUTO & trajectory
  //we probably should put units on all of these variable names for clarity
  public static final double gearboxRatio             = 8.783;                             //it probably is not this it just is the most accurate with this value
  public static final double positionConversionFactor = 0.1524*Math.PI/gearboxRatio;      //changes rotations of motor to distance traveled in meters
  public static final double velocityConversionFactor = Math.PI*0.1524/60/gearboxRatio;   //changes rpm of motor to m/s
  public static final double kTrackwidth              = 0.6589; //idk if this is right lole
  public static final DifferentialDriveKinematics kDriveKinematics = 
    new DifferentialDriveKinematics(kTrackwidth);
  public static final double kMaxVelocity     = 0.5;
  public static final double kMaxAcceleration = 1.0;
  public static final double kRamseteB        = 2.0;
  public static final double kRamseteZeta     = 0.7;
  public static final double autoMaxVoltage   = 8.0;

  // PID/FEEDFORWARD VALUES
  // AUTO
    public static final double ksAuto = 0.18734;
    public static final double kvAuto = 2.1245;
    public static final double kaAuto = 0.48883;
    public static final double kpAuto = 2.6983;
  // SHOOTER
    public static final double kpShooter = 0.95; //0.65;
    public static final double kiShooter = 0.0;
    public static final double kdShooter = 0.05;
    public static final double kfShooter = 0.975; //1.0;



  // VISION
  // Measurements for limelight
    public static final double targetHeight = 104.0; //TODO: CHANGE THIS TO 104 AT COMPETITION
    public static final double limelightHeight = 32.0; //estimate
    public static final double limelightAngle  = 25.0;
    public static final double limelightDistToFront = 18.0;
  
    // limelight PID values
    public static final double lookToTargetP = 0.05;
    public static final double lookToTargetI = 0.00;
    public static final double lookToTargetD = 0.00;

  // SHOOTER
  // TODO: ensure shooter doesnt overshoot at comp
  // Flywheel
    public static final double flywheelM = 6.026;
    public static final double flywheelB = 2514;

  // Backspinner
    public static final double backspinnerM = 5.98;
    public static final double backspinnerB = 2711;
  
  // Indexer
    public static final double indexerSetpoint = 1000; // its like 200 rpm lower in practice because i didnt have time to tune pid lol

    public static final double indexerP = 1.25;
    public static final double indexerI = 0;
    public static final double indexerD = 0;
    
  // MOTOR VOLTAGE PERCENTAGES
  // Climb

  // Drivetrain

  // Intake
    public static final double intakeMotorVoltage   = 1.0;
    public static final double kowalskiMotorVoltage = 0.5;
  // Shooter
    



  // MOTOR & SOLENOID IDs
  // Drivetrain
    public static final int l1MotorID = 1;
    public static final int l2MotorID = 2;
    public static final int r1MotorID = 3;
    public static final int r2MotorID = 4;
  // Intake
    public static final int intakeMotorID = 5;
    public static final int kowalksiMotorID = 6; // da kowalksi :D
  // Shooter
    public static final int indexerMotorID = 7;
    public static final int flywheelMotorID = 8;
    public static final int backspinnerMotorID = 9;
  // Climb
    public static final int armMotorID = 10;
  // SOLENOIDS
    public static final int armPistonFowardID = 0;
    public static final int armPistonReverseID = 1;
    public static final int shooterPistonReverseID = 2;
    public static final int shooterPistonForwardID = 3;
    public static final int leftIntakePistonFowardID = 4;
    public static final int leftIntakePistonReverseID = 5;



  // BUTTON IDS
    // Camera
      public static final int turnToTargetButtonID = 8;
    // Climb
      public static final int manualMoveArmButtonID = 11;
      public static final int climbRoutineButtonID = 12;
      //needs to be implemented
      public static final int manualArmPneumaticButtonID = 7;
      public static final int fullExtendButtonID = 10;
      public static final int fullRetractButtonID = 9;
    // Drivetrain

    // Intake
      public static final int intakeBallsButtonID = 1;
      public static final int reverseIntakeButtonID = 5;
    // Shooter     
      public static final int closeShootButtonID = 2;
      public static final int farShootButtonID = 3;
      public static final int shooterToggleButtonID = 6;
}