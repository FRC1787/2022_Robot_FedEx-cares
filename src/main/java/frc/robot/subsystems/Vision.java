package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

  private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  NetworkTableEntry proximityEntry;
  NetworkTableEntry colorEntry;

  //tune these values
  private final Color kRedTarget = new Color(0.48, 0.39, 0.13);
  private final static Color kBlueTarget = new Color(0.2, 0.38, 0.42);


  private final static ColorMatch m_colorMatch = new ColorMatch();

  private static Color color;
  private double proximity;

  static String allianceColor;


  private static double distToTarget = 0;
  private static double x;
  private static double y;

  // private static UsbCamera camera = new UsbCamera(name, path);

  public Vision() {
    if(DriverStation.getAlliance().equals(Alliance.Blue)) {
      allianceColor = "blue";
    }else {
      allianceColor = "red";
    }

    m_colorMatch.addColorMatch(kBlueTarget);
    m_colorMatch.addColorMatch(kRedTarget);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    colorEntry = inst.getEntry("/rawcolor1");
    proximityEntry = inst.getEntry("/proximity1");
  
    // // make sure this doesnt break stuff
    // CameraServer.startAutomaticCapture();
    // CvSink cvSinkVideo = CameraServer.getVideo();
    // CvSource outputVideo = CameraServer.putVideo("USB Camera", Constants.cameraWidth, Constants.cameraHeight);
  }

  private double[] getNormalizedColors(double[] rawColors) {
    double sum = rawColors[0] + rawColors[1] + rawColors[2];

    double[] ret = {rawColors[0] / sum, rawColors[1] / sum, rawColors[2] / sum};
    return ret;
}

  public static String getPredictedColor() {
    ColorMatchResult match = m_colorMatch.matchClosestColor(color);

    if(match.color == kBlueTarget) {
        return "blue";
    }
    return "red";
  }

  public static boolean isOppositeColor() {
    
    return !getPredictedColor().equals(allianceColor);

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

    // Colorsensor stuff
    double[] rawColors = colorEntry.getDoubleArray(new double[4]);
    double[] normColors = getNormalizedColors(rawColors);
    
    color = new Color(normColors[0], normColors[1], normColors[2]);
    proximity = proximityEntry.getDouble(0.0);

   

    SmartDashboard.putString("color", getPredictedColor());
  }


}