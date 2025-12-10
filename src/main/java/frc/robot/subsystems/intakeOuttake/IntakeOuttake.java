package frc.robot.subsystems.intakeOuttake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.IntakeOuttakeConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeOuttake implements IntakeOuttakeIO {

    private IntakeIO io;
    private final IntakeOuttakeIOInputsAutoLogged inputs = new IntakeOuttakeIOInputsAutoLogged();
  
    private PIDController controller = new PIDController(0.0, 0.0, 0.0);
    private final SimpleMotorFeedforward ffModel;
  
    private static double setpointRPMs = 0.0;
    private static double setpointRadPerSec = 0.0;

    public IntakeOuttake(IntakeOuttakeIO io) {
        System.out.println("[Init] Creating Intake");
        this.io = io;
        io.setBrakeMode(false);
    
        controller.setPID(IntakeOuttakeConstants.kP, IntakeOuttakeConstants.kI, 0.0);
    
        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Constants.getRobot()) {
          case Constants.RobotType.ROBOT_REAL:
            ffModel = new SimpleMotorFeedforward(0.1, 0.12);
            break;
          case Constants.RobotType.ROBOT_SIM:
          default:
            ffModel = new SimpleMotorFeedforward(0.0, 0.09);
            break;
        }
      }
      private static enum IntakeOuttakeMode {
        STOPPED,
        GO
      };
    
      private IntakeOuttakeMode mode = IntakeOuttakeMode.STOPPED;

    public void intake() {
      io.setVoltage(IntakeOuttakeConstants.INTAKE_VOLTAGE);
    }  

    public void outtake() {
      io.setVoltage(IntakeOuttakeConstants.OUTTAKE_VOLTAGE);
    }

    public void periodic() {

      if (DriverStation.isDisabled()) {
        io.setVoltage(0.0);
        controller.reset();
        mode = IntakeOuttakeMode.STOPPED;
      }

        io.updateInputs(inputs);
    }
    
    public boolean hasGamePiece() {
     return inputs.hasGamePiece;
    }
    
    public boolean hasNoGamePiece() {
        return !inputs.hasGamePiece;
    }

    public void go() {
      mode = IntakeOuttakeMode.GO;
    }
  
    public void stop() {
      mode = IntakeOuttakeMode.STOPPED;
    }

      @AutoLogOutput
    public double getSetpointRadians() {
        return setpointRadPerSec;
    }
    
      @AutoLogOutput
    public double getFF() {
        return ffModel.calculate(setpointRadPerSec);
    }
    
      @AutoLogOutput
    public double getPID() {
        return controller.calculate(inputs.velocityRadPerSec);
    }
    
    @AutoLogOutput
    public IntakeOuttakeMode getState() {
        return mode;
    }
    
      @AutoLogOutput
    public double getSetPointRPMs() {
        return setpointRPMs;
    }
    
      @AutoLogOutput
    public double getVelocityRPMs() {
        return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    }
    
}