package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private static double distToTarget = 0;
  private static double x;
  private static double y;

  // private static UsbCamera camera = new UsbCamera(name, path);

  public Vision() {
  
    // // make sure this doesnt break stuff
    // CameraServer.startAutomaticCapture();
    // CvSink cvSinkVideo = CameraServer.getVideo();
    // CvSource outputVideo = CameraServer.putVideo("USB Camera", Constants.cameraWidth, Constants.cameraHeight);
  }

  /**
   * Returns the x value of any target seen by the Limelight
   * 
   * @return x value of target
   */
  public static double getLimelightX() {
    return table.getEntry("tx").getDouble(0.0);
  }

  public static double getLimelightA() {
    return table.getEntry("ta").getDouble(0.0);
  }

  /**
   * Returns the current mode of the Limelight's LEDs
   * 
   * @return current mode of the Limelight's LEDs
   */
  public static int getCameraLightState() {
    return (int) table.getEntry("ledMode").getDouble(3);
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
    return (Constants.targetHeight - Constants.limelightHeight)/(Math.tan(Math.toRadians(y + Constants.limelightAngle)))-Constants.limelightDistToFront;
  }

  @Override
  public void periodic() {

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    distToTarget = (Constants.targetHeight - Constants.limelightHeight)/(Math.tan(Math.toRadians(y + Constants.limelightAngle)))-Constants.limelightDistToFront;

    SmartDashboard.putNumber("Limelight Distance", distToTarget);
  }
}