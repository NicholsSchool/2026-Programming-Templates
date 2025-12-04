package frc.robot.subsystems.IntakeOuttake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class IntakeOuttake implements IntakeOuttakeIO {

    private IntakeIO io;
    private final IntakeOuttakeIOInputsAutoLogged inputs = new IntakeOuttakeIOInputsAutoLogged();
  
    private PIDController controller = new PIDController(0.0, 0.0, 0.0);
    private final SimpleMotorFeedforward ffModel;
  
    private static double setpointRPMs = 0.0;
    private static double setpointRadPerSec = 0.

    public IntakeOuttake(IntakeOuttakeIO io) {
        System.out.println("[Init] Creating Intake");
        this.io = io;
        io.setBrakeMode(false);
    
        controller.setPID(kP.get(), kI.get(), 0.0);
    
        // Switch constants based on mode (the physics simulator is treated as a
        // separate robot with different tuning)
        switch (Constants.getRobot()) {
          case ://ROBOT_REAL:
            ffModel = new SimpleMotorFeedforward(0.1, 0.12);
            break;
          case ://ROBOT_SIM:
          default:
            ffModel = new SimpleMotorFeedforward(0.0, 0.09);
            break;
        }
      }
    

    public void intake() {

    }  

    public void outtake() {

    }

    public void setVoltage(){

    }

    public void getVoltage(){

    }

    public void periodic() {

        io.updateInputs(inputs);
    }
    
    public boolean hasGamePiece() {
     return inputs.hasGamePiece;
    }
    
    public boolean hasNoGamePiece() {
        return !inputs.hasGamePiece;
    }

    public void stop() {
        mode = IntakeMode.kStopped;
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
    public IntakeMode getState() {
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