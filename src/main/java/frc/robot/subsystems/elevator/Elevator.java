package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.LoggedTunableNumber;


public class Elevator extends SubsystemBase
{
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private ElevatorIO io;


    private double accelRad = 0.0;

    private double targetHeight = 0.0;
    private double voltageCmdPid = 0.0;
    private boolean reachedTargetPosition = true;
    private double voltageCmdManual = 0.0;
    private boolean hasHitLimitSwitch = false;
    private boolean isAuto = false;

    private enum ElevatorMode{
      MANUAL,
      GO_TO_POSITION
    }

    private ElevatorMode elevatorMode;

    private final ProfiledPIDController elevatorPidController =
    new ProfiledPIDController(
        0,0 ,0, new TrapezoidProfile.Constraints(0,0));

        
  private static final LoggedTunableNumber elevatorMaxVelocityRad =
    new LoggedTunableNumber("elevator/MaxVelocityRad");
  private static final LoggedTunableNumber elevatorMaxAccelerationRad =
    new LoggedTunableNumber("elevator/MaxAccelerationRad");
  private static final LoggedTunableNumber elevatorKp = new LoggedTunableNumber("elevator/Kp");
  private static final LoggedTunableNumber elevatorKi = new LoggedTunableNumber("elevator/Ki");
  private static final LoggedTunableNumber elevatorKd = new LoggedTunableNumber("elevator/Kd");

    public Elevator(ElevatorIO io){
        this.io = io;

        reachedTargetPosition = true;

        elevatorMaxAccelerationRad.initDefault(Constants.ElevatorConstants.ELEVATOR_MAX_ACCELERATION_RAD);
        elevatorMaxVelocityRad.initDefault(Constants.ElevatorConstants.ELEVATOR_MAX_VELOCITY_RAD);


        elevatorKp.initDefault(ElevatorConstants.ELEVATOR_P);
        elevatorKi.initDefault(ElevatorConstants.ELEVATOR_I);
        elevatorKd.initDefault(ElevatorConstants.ELEVATOR_D);

        elevatorPidController.setP(elevatorKp.get());
        elevatorPidController.setI(elevatorKi.get());
        elevatorPidController.setD(elevatorKd.get());

        this.elevatorMode = ElevatorMode.GO_TO_POSITION;
        setTargetPosition(getHeight());
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);

        updateTunables();

        Logger.processInputs("Elevator", inputs);

        switch(elevatorMode){
          case GO_TO_POSITION:
          voltageCmdPid = (hasHitLimitSwitch || isAuto) ? elevatorPidController.calculate( this.getHeight()) : 0.0;
          voltageCmdManual = 0.0;
          break;
          case MANUAL:
          voltageCmdPid = 0.0;
          setTargetPosition(getHeight());
          }

         if (!reachedTargetPosition) {
            reachedTargetPosition = elevatorPidController.atGoal();
            if (reachedTargetPosition) System.out.println("elevator Move to Pos Reached Goal!");
          }

          if(inputs.limitSwitch){
            hasHitLimitSwitch = true;
          }
        
          io.setVoltage(voltageCmdManual + voltageCmdPid);
    }


    private void updateTunables() {
        // Update from tunable numbers
        if (elevatorMaxVelocityRad.hasChanged(hashCode())
            || elevatorMaxAccelerationRad.hasChanged(hashCode())
            || elevatorKp.hasChanged(hashCode())
            || elevatorKi.hasChanged(hashCode())
            || elevatorKd.hasChanged(hashCode())) {
          elevatorPidController.setP(elevatorKp.get());
          elevatorPidController.setI(elevatorKi.get());
          elevatorPidController.setD(elevatorKd.get());
          elevatorPidController.setConstraints(
              new TrapezoidProfile.Constraints(elevatorMaxVelocityRad.get(), elevatorMaxAccelerationRad.get()));
        }
      }

      public void setTargetPosition(double targetHeight) {
        this.targetHeight = targetHeight;
        elevatorPidController.setGoal((targetHeight));
        elevatorPidController.reset(inputs.currentHeight);
        reachedTargetPosition = false;
      }

      // Checks if TargetHeight is valid
        public Command runGoToPositionCommand(double targetHeight){
    elevatorMode = ElevatorMode.GO_TO_POSITION;
    if ((targetHeight > ElevatorConstants.ELEVATOR_MAX_HEIGHT || targetHeight < ElevatorConstants.ELEVATOR_MIN_HEIGHT) && hasHitLimitSwitch) {
      System.out.println("Soft Limited Elevator");
      return new InstantCommand();
    }
    System.out.println("Setting go to pos:" + targetHeight );
    isAuto = DriverStation.isAutonomous();
    return new InstantCommand(() -> setTargetPosition(targetHeight), this);
  }

    /**
     * When the Joystick passes the Joystick Deadband threshold
     * the elevator is set to manual else it is go to position
    */
      public void runManualPosition(double stickPosition){
        if(Math.abs(stickPosition) > Constants.JOYSTICK_DEADBAND){
          elevatorMode = ElevatorMode.MANUAL;
        }
        else{
          elevatorMode = ElevatorMode.GO_TO_POSITION;
          }
        }
        
       /** 
        * Autonomous Log Outputs 
        */ 
  @AutoLogOutput
  public ElevatorMode getElevatorMode(){
    return elevatorMode;
  }

  @AutoLogOutput
  public double getHeight(){
    return inputs.currentHeight;
  }

  @AutoLogOutput
  public double targetHeight(){
    return targetHeight;
  }

  @AutoLogOutput
  public double getAcceleration() {
    return accelRad;
  }

  @AutoLogOutput
  public double getVoltageCommandPid() {
    return voltageCmdPid;
  }

  @AutoLogOutput
  public double[] getOutputCurrent() {
    return inputs.currentAmps;
  }

  @AutoLogOutput
  public boolean isAtGoal() {
    return elevatorPidController.atGoal();
  }
  
}