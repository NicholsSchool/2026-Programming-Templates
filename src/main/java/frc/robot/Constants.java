package frc.robot;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static final RobotType robot = RobotType.ROBOT_REAL;
  public static final boolean driveRobotRelative =
      false; // set to true to override all field relative and instead command in robot-relative.

  // true to place tunable numbers in dashboard for setting, false otherwise
  public static final boolean tuningMode = true;
  public static final double loopPeriodSecs = 0.02;
  public static final double MeterPerInch = 0.0254;
  public static final double KgPerLb = 0.453592;

  public static final double JOYSTICK_DEADBAND = 0.08;

  public static RobotType getRobot() {
    return RobotBase.isReal() ? robot : RobotType.ROBOT_SIM;
  }

  public static enum RobotType {
    ROBOT_REAL_FRANKENLEW, // a real robot (LEW ZEALAND)
    ROBOT_REAL, // a real robot (JANICE)
    ROBOT_REPLAY, // data file replay (could be on real bot or simulation)
    ROBOT_SIM, // simulation
    ROBOT_FOOTBALL, // Football for simulating
    ROBOT_CALIBRATE
  }

  // CAN IDs (Controller Area Network)
  public static final class CAN {

  }

  public static final class RobotConstants {
   
  }

  public static final class DriveConstants {
   
  }

  // REV MAXSwerve Modules
  public static final class ModuleConstants {
   
  }

  public static final class ElevatorConstants{
  
  }

  public static final class IntakeOuttakeConstants {

  }

  public static final class ShooterConstants{
    public static final double SHOOTER_RPM = 3000;
    public static final double REVERSE_RPM = -3000;
    public static final double P = 1.0; //This is a constant for position control
    public static final double D = 0.0; //This is a constant for position control
  }

  public static final class IndexerConstants{
    //TODO: Tune
    public static double kP = 2.54;
    public static double kI = 0.00;
    public static double kD = 0.003;

    public static double kMaxAccel = 1; // In/Sec/Sec
    public static double kMaxVel = 10; // In/Sec

    public static double shiftLengthIN = 12; //How much to shift by every time. Usually close to the game piece width.
    public static double inchesPerRadian = 2 / Math.PI; // Inches of shift per radian of rotation. (Wheel Circumference / 2pi)
    public static double gearRatio = 1.0;
    public static double baseVoltage = 12.0; // Maximum voltage that the manual command will run at.
  }

  public static final class ArmConstants{
    
  }


}
