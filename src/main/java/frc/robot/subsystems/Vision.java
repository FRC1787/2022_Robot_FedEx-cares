package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoCamera.WhiteBalance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Robot;

public class Vision extends SubsystemBase {


  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public static double distToTarget = 0;
  public static double x, y;

  // private UsbCamera powerCellCam;

  // private CvSource outputStream;

  // private CvSink powerCellFrameGrabber;

  public Vision() {

  //   //Initialize each camera with a channel and name, pushes non-processed images
  //   powerCellCam = CameraServer.startAutomaticCapture("Camera", 0);

  //   //Configure resoltuion, FPS, exposure, brightness and white-balance
  //   configureCamera(powerCellCam, false);

  //   //Initialize frame grabber used to grab individual frames from video stream to be processed later
  //   powerCellFrameGrabber = CameraServer.getVideo(powerCellCam);

  //   //Push processed or unprocessed frames
  //   outputStream = CameraServer.putVideo("Processed Video", 160, 120);
  
  // }

  // public void configureCamera(UsbCamera camera, boolean targetingCamera) {
  //   camera.setResolution(160, 120);
  //   camera.setFPS(15);
  //   camera.setExposureAuto();
  //   camera.setBrightness(40);
  //   camera.setWhiteBalanceManual(WhiteBalance.kFixedIndoor);
  }

  /**
   * Returns the x value of any target seen by the Limelight
   * 
   * @return x value of target
   */
  public static double getLimelightX() {
    return table.getEntry("tx").getDouble(0.0);
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
    //find regression formula here
    if (Shooter.isRaised) {
      if (y < -17) return 3600; //launchpad shot
      if (Robot.inAuto) return 3000; //auto shot
      else return 2900; //tarmac shot
    }
    else return 2800; //up close shot
  }

  public static double calculateBackspinnerRPM() {
    if (Shooter.isRaised) {
      if (y < -17) return 3600;
      if (Robot.inAuto) return 3200;
      else return 3100;
    }
    else return 3000;
  }

  // public static double calculateShooterPosition() {
  //   // if ()
  // }

  @Override
  public void periodic() {

    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    distToTarget = ((Constants.tapeHeight - Constants.limelightHeight)/(Math.tan(Math.toRadians(y + Constants.limelightAngle))));

    SmartDashboard.putNumber("Limelight Distance", distToTarget);
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

  }
}