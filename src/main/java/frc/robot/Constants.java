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
  public static final boolean botMostLikelyToGetPHD        = true;
  public static final boolean botWithWorstPathweaverSystem = true;
  public static final boolean FedEx_cares                  = true;
  public static final boolean bestSchoolTeamAtOrange       = true;

  // SHOOTER
  public static double shooterDistanceThreshold = 99999;
  public static double acceleratorRPMToPercent = 1./5000.; //DO THIS
  public static double backspinnerRPMToPercent = 1./1200.;


  // SOLENOID IDs
    // Climb
      public static final int armPistonFowardID  = 0;
      public static final int armPistonReverseID = 1;
    // Intake
      public static final int leftIntakePistonFowardID   = 4;
      public static final int leftIntakePistonReverseID  = 5;
    // Shooter
      public static final int shooterPistonForwardID = 3;
      public static final int shooterPistonReverseID = 2;






  // MOTOR IDs

    // Drivetrain
      public static final int l1MotorID = 1;
      public static final int l2MotorID = 2;
      public static final int r1MotorID = 3;
      public static final int r2MotorID = 4;
    // Intake
      public static final int intakeMotorID   = 5; //change this later
      public static final int kowalksiMotorID = 6; // da kowalksi :D
    // Shooter
      public static final int indexerMotorID     = 7;
      public static final int acceleratorMotorID = 8;
      public static final int backspinnerMotorID = 9;
    // Climb
      public static final int armMotorID = 10;
  






  // MOTOR SPEEDS
    // Climb

    // Drivetrain

    // Intake
      public static final double intakeMotorSpeed   = 1.0;
      public static final double kowalskiMotorSpeed = 0.5;

    // Shooter
      public static final double indexerSpeed     = 0.20;
      //make this always negative pls
      public static final double acceleratorSpeed = -0.52;
      public static final double backspinnerSpeed = 0.52;



  // PID/FEEDFORWARD VALUES
    // AUTO
      public static final double ksAuto = 0.090934;
      public static final double kvAuto = 1.918800;
      public static final double kaAuto = 0.419780;
      public static final double kpAuto = 2.648400;

    // SHOOTER
      public static final double kpShooter = 0.05;
      public static final double kiShooter = 0.000000;
      public static final double kdShooter = 0.000000;
      




  //AUTO
  //we probably should put units on all of these variable names for clarity
    public static final double gearboxRatio             = 7.25;                             //it probably is not this it just is the most accurate with this value
    public static final double positionConversionFactor = 0.1524*Math.PI/gearboxRatio;      //changes rotations of motor to distance traveled in meters
    public static final double velocityConversionFactor = Math.PI*0.1524/60/gearboxRatio;   //changes rpm of motor to m/s

    public static final double kTrackwidth = 0.66; //idk if this is right lole
    public static final DifferentialDriveKinematics kDriveKinematics = 
      new DifferentialDriveKinematics(kTrackwidth);
  
    public static final double kMaxSpeed        = 0.5;
    public static final double kMaxAcceleration = 1.0;

    public static final double kRamseteB    = 2.0;
    public static final double kRamseteZeta = 0.7;

    public static final double autoMaxVoltage = 8.0;





  // VISION
    // Measurements for limelight
      public static final double tapeHeight      = 81.0;
      public static final double limelightHeight = 46.0;
      public static final double limelightAngle  = 25.0;
    // limelight PID values
      public static final double lookToTargetP = 0.05;
      public static final double lookToTargetI = 0.00;
      public static final double lookToTargetD = 0.00;






  // BUTTON IDS
    // Camera
      public static final int toggleLimelightButtonID = 12;
      public static final int turnToTargetButtonID    =  6;

    // Climb
      public static final int manualMoveArmButtonID = 7;
      public static final int climbRoutineButtonID  = 8;
      public static final int fullExtendButtonID    = 9;


    // Drivetrain

    // Intake
      public static final int intakeBallsButtonID = 1;
      public static final int reverseIntakeButtonID = 6;

    // Shooter
      public static final int basicShootButtonID = 2;
      public static final int smartShootButtonID = 3;
      






  
}
