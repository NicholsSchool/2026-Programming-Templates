package frc.robot.subsystems.arm;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;




public class Arm extends SubsystemBase {
    private ArmIO io;
    private ArmState state;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private ArmFeedforward armFF = new ArmFeedforward(Constants.ArmConstants.ARM_FF_KS, Constants.ArmConstants.ARM_FF_KG, Constants.ArmConstants.ARM_FF_KV, Constants.ArmConstants.ARM_FF_KA);
    private double voltageCmdFF = 0.0;
    private double voltageCmdPid = 0.0;
    private double targetAngleDeg = 0.0;
    private double manualInput = 0.0;
    private double voltageCommand = 0.0;
    private double previousVoltage = 0.0;

    private boolean reachedTargetPos = false;

    private final ProfiledPIDController PIDcontroller = new ProfiledPIDController(Constants.ArmConstants.ARM_P, Constants.ArmConstants.ARM_I, Constants. ArmConstants.ARM_D, 
    new TrapezoidProfile.Constraints(0, 0));

    private static final LoggedTunableNumber armKg = new LoggedTunableNumber("Arm/kG");
    private static final LoggedTunableNumber armKv = new LoggedTunableNumber("Arm/kV");
    private static final LoggedTunableNumber armKa = new LoggedTunableNumber("Arm/kA");

    private static final LoggedTunableNumber armKp = new LoggedTunableNumber("Arm/kP");
    private static final LoggedTunableNumber armKi = new LoggedTunableNumber("Arm/kI");
    private static final LoggedTunableNumber armKd = new LoggedTunableNumber("Arm/kD");

    private static final LoggedTunableNumber positionToleranceDeg =
      new LoggedTunableNumber("Arm/PositionToleranceDeg");
    private static final LoggedTunableNumber armMaxAccelerationRad =
        new LoggedTunableNumber("Arm/armMaxAccelerationRad");
    private static final LoggedTunableNumber armMaxVelocityRad =
        new LoggedTunableNumber("Arm/MaxVelocityRad");
    


    private enum ArmState {
        MANUAL,
        POS
    }

    private ArmState armState;

    public Arm (ArmIO io){
        this.io = io;
        armState = ArmState.POS;

        armKg.initDefault(Constants.ArmConstants.ARM_FF_KG);
        armKv.initDefault(Constants.ArmConstants.ARM_FF_KV);
        armKa.initDefault(Constants.ArmConstants.ARM_FF_KA);

        armKp.initDefault(Constants.ArmConstants.ARM_P);
        armKi.initDefault(Constants.ArmConstants.Arm_I);
        armKd.initDefault(Constants.ArmConstants.ARM_D);
        armMaxAccelerationRad.initDefault(Constants.ArmConstants.ARM_MAX_ACCEL);
        armMaxVelocityRad.initDefault(Constants.ArmConstants.ARM_MAX_VEL);
        positionToleranceDeg.initDefault(2.0);

        PIDcontroller.setP(armKp.get());
        PIDcontroller.setI(armKi.get());
        PIDcontroller.setD(armKd.get());
        new TrapezoidProfile.Constraints(armMaxVelocityRad.get(), armMaxAccelerationRad.get());
        PIDcontroller.setTolerance(Units.degreesToRadians(positionToleranceDeg.get()));
    }


    @Override
    public void periodic(){

        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
        updateTunables();

        voltageCmdFF = armFF.calculate(inputs.angleRads, Constants.ArmConstants.ARM_MAX_VEL * -1
        //manual);
        );
        switch(state){
            case MANUAL:
            break;
            case POS:
            break;
        } 

        if (DriverStation.isDisabled()) {}

        switch (armState) {
          case MANUAL:
            voltageCmdPid = 0.0;
            voltageCmdFF =
                armFF.calculate(
                    inputs.angleRads, Constants.ArmConstants.ARM_MAX_VEL * softLimit(manualInput));
            break;
          case kGoToPos:
            voltageCmdPid = PIDcontroller.calculate(inputs.angleRads);
            voltageCmdFF = armFF.calculate(inputs.angleRads, PIDcontroller.getSetpoint().velocity);
    
            if (!reachedTargetPos) {
              reachedTargetPos = PIDcontroller.atGoal();
              if (reachedTargetPos) {
                System.out.println("Arm Move To Pos Reached Goal!");
                timerMoveToPose.stop();
              }
            }
            break;
        }
            voltageCommand = voltageCmdPid + voltageCmdFF;
            io.setVoltage(voltageCommand);
    }

    public void setTargetangle(double targetAngleDeg){
        this.targetAngleDeg = targetAngleDeg;

    }

    public void setGoToPosition(){
        this.manualInput = 0.0;
        armState = ArmState.POS;
    }

    private void updateTunables() {
        // Update from tunable numbers
        if (armKg.hasChanged(hashCode())
            || armKv.hasChanged(hashCode())
            || armKa.hasChanged(hashCode())) {
          ARM_FF = new ArmFeedforward(0.0, armKg.get(), armKv.get(), armKa.get());
        }
        if (armMaxVelocityRad.hasChanged(hashCode())
            || armMaxAccelerationRad.hasChanged(hashCode())
            || positionToleranceDeg.hasChanged(hashCode())
            || armKp.hasChanged(hashCode())
            || armKi.hasChanged(hashCode())
            || armKd.hasChanged(hashCode())) {
          armPidController.setP(armKp.get());
          armPidController.setI(armKi.get());
          armPidController.setD(armKd.get());
          armPidController.setConstraints(
              new TrapezoidProfile.Constraints(armMaxVelocityRad.get(), armMaxAccelerationRad.get()));
          armPidController.setTolerance(Units.degreesToRadians(positionToleranceDeg.get()));
        }
      }
    


}
