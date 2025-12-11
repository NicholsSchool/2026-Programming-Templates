package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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

    public static final double ELEVATOR_MAX_ACCELERATION_RAD = 0;
    public static final double ELEVATOR_MAX_VELOCITY_RAD = 0;
    public static final double ELEVATOR_P = 0;
    public static final double ELEVATOR_I = 0;
    public static final double ELEVATOR_D = 0;
    public static final double ELEVATOR_MAX_HEIGHT = 0;
    public static final double ELEVATOR_MIN_HEIGHT = 0;
  
  }

  public static final class IntakeOuttakeConstants {

  }

  public static final class ShooterConstants{
    public static final double SHOOTER_RPM = 1;
    public static final double REVERSE_RPM = 1;
    public static final double DELIVER_RPM = 1;
    public static final double P = 1; //This is a constant for position control
    public static final double D = 1; //This is a constant for position control
  }

  public static final class IndexerConstants{

  }

  public static final class ArmConstants{
    
  }


}
