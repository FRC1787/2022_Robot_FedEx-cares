package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  private static PhotonCamera limelight = new PhotonCamera("ballcamera");;
  private static PhotonPipelineResult result;
  private static double yaw, pitch, area;

  public Vision() {
  

  }

  public static boolean hasTargets() {
    return result.hasTargets();
  }
  
  public static double getLimelightX() {
    return yaw;
  }

  public static double calculateFlywheelRPM() {
  
    if (Shooter.isRaised)  
      return (Constants.flywheelM * limelightDistance()) + Constants.flywheelB;
    else return 2850;

    // if (Shooter.isRaised) {
    //   if (Robot.inAuto) return 2950; //auto shot
    //   return 3600; //launchpad shot
    //   //else return 3100; //tarmac shot
    // }
    // return 2850; //up close shot
  }

  public static double calculateBackspinnerRPM() {
    if (Shooter.isRaised)
      return (Constants.backspinnerM * limelightDistance()) + Constants.backspinnerB;
    else return 3000;

    // if (Shooter.isRaised) {
    //   if (Robot.inAuto) return 3150;
    //   return 3750;
    //   //else return 3200;
    // }
    // return 3000;
  }

  public static double limelightDistance() {
    return PhotonUtils.calculateDistanceToTargetMeters(
      Constants.limelightHeightMeters,
      Constants.targetHeightMeters, 
      Math.toRadians(Constants.limelightAngle), 
      Math.toRadians(pitch)
    );
  }

  public static double limelightArea() {
    return area;
  }

  @Override
  public void periodic() {
    result = limelight.getLatestResult();
    if (result.hasTargets()) {
      yaw = result.getBestTarget().getYaw();
      pitch = result.getBestTarget().getPitch();
      area = result.getBestTarget().getArea();
    }
    else {
      yaw = 0; pitch = 0; area = 0;
    }

    SmartDashboard.putNumber("limelight distance", limelightDistance());

  }
}