package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterIO; 
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import frc.robot.Constants.ShooterConstants;



public class Shooter extends SubsystemBase {
  private double setpoint;
  private ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  public double voltageCommand;


  public static final LoggedTunableNumber shootVelocity =
      new LoggedTunableNumber("Shooter/AmpVelocityRPMs");
  public static final LoggedTunableNumber deliverVelocity =
      new LoggedTunableNumber("Shooter/SpeakerVelocityRPMs");
  public static final LoggedTunableNumber reverseVeloctiy =
      new LoggedTunableNumber("Shooter/TrapVelocityRPMs");
  public static final LoggedTunableNumber spinDurationSec =
      new LoggedTunableNumber("Shooter/SpinDurationSec");
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP");
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Shooter/kD");


  private PIDController controller = new PIDController(0.0, 0.0, 0.0);
  private final SimpleMotorFeedforward ffModel;


  private static enum ShooterMode {
    kStopped,
    kShoot,
    kReverse
  };


  private ShooterMode mode = ShooterMode.kStopped;


  public Shooter(ShooterIO io) {
    System.out.println("[Init] Creating Shooter");
    this.io = io;


    // Sets the default using ShooterConstants // MAKE SHOOTER CONSTANTS!!! Simply fill-in
    shootVelocity.initDefault(Constants.ShooterConstants.SHOOTER_RPM);
    reverseVeloctiy.initDefault(Constants.ShooterConstants.REVERSE_RPM);
    spinDurationSec.initDefault(1.5);
    kP.initDefault(Constants.ShooterConstants.P);
    kD.initDefault(Constants.ShooterConstants.D);


    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.getRobot()) {
      case ROBOT_REAL:
        ffModel = new SimpleMotorFeedforward(0.001, 0.0060);
        break;
      case ROBOT_SIM:
      default:
        ffModel = new SimpleMotorFeedforward(1, 1);
        break;
    }
  }


  @Override
  public void periodic() {
    setpoint = 0.0;
    io.updateInputs(inputs);
    Logger.processInputs("shooter", inputs);


    // Update tunable numbers
    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      controller.setP(kP.get()); //kD, and kP may also be K and D inside shooter constants
      controller.setD(kD.get());
    }


    // Reset when disabled
    if (DriverStation.isDisabled()) {
      controller.reset();
      mode = ShooterMode.kStopped;
    } else {
      switch (mode) {
        case kShoot:
          setpoint = shootVelocity.get();
          break;
        case kReverse:
          setpoint = reverseVeloctiy.get();
          break;
        case kStopped:
        default:
          setpoint = 0.0;
      }


      voltageCommand = controller.calculate(inputs.velocityRPMs, setpoint);
      io.setVoltage(MathUtil.clamp(voltageCommand, -12.0, 12.0));
    }
  }

  public void setShoot() {
    mode = ShooterMode.kShoot;
  }


  public void setReverse() {
    mode = ShooterMode.kReverse;
  }


  public void stop() {
    mode = ShooterMode.kStopped;
  }


  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }


  @AutoLogOutput
  public double getSetpointVelocityRPMs() {
    return setpoint;
  }


  @AutoLogOutput
  public double getVoltageCommand() {
    return voltageCommand;
  }


  @AutoLogOutput
  public double getActualVelocityRPMs() {
    return inputs.velocityRPMs;
  }
}

