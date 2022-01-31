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
    public static final boolean botMostLikelyToGetPHD = true;
    public static final boolean botWithWorstPathweaverSystem = true;

    //we probably should put units on all of these variable names for clarity

    public static final double gearboxRatio = 7.25;
    public static final double wheelDiameter = 6.0;
    public static final int pulsesPerRotation = 42;

    public static final double positionConversionFactor = 0.1524*Math.PI/gearboxRatio; //changes rotations of motor to distance traveled in meters
    public static final double velocityConversionFactor = Math.PI*0.1524/60/gearboxRatio; //changes rpm of motor to m/s


    public static final double ks = 0.090934;
    public static final double kv = 1.9188;
    public static final double ka = 0.41978;

    public static final double kp = 2.6484;

    public static final double kTrackwidth = 0.66; //idk if this is right lole
    public static final DifferentialDriveKinematics kDriveKinematics = 
        new DifferentialDriveKinematics(kTrackwidth);
    
    public static final double kMaxSpeed = 0.2;
    public static final double kMaxAcceleration = 1.0;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double autoMaxVoltage = 8;

    // Measurements for limelight
    public static final double tapeHeight = 81;
    public static final double limelightHeight = 46;
    public static final double limelightAngle = 25;


    // button ids
    public static final int toggleLimelightButtonID = 12;
    public static final int lookToTargetButtonID = 5;
}
