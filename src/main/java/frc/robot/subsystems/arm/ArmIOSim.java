package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO{
    private static final DCMotor lSimModel = DCMotor.getKrakenX60(1);
    private static final DCMotor rSimModel = DCMotor.getKrakenX60(1);

    private final DCMotorSim lSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(lSimModel, 0.025, Constants.ArmConstants.ARM_GEAR_RATIO),
          lSimModel);
    private final DCMotorSim rSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(rSimModel, 0.025, Constants.ArmConstants.ARM_GEAR_RATIO),
          rSimModel);

    @Override
    public void updateInputs(ArmIOInputs inputs){
        lSim.update(Constants.loopPeriodSecs);
        rSim.update(Constants.loopPeriodSecs);
              
        inputs.appliedVoltage = new double[] {lSim.getInputVoltage(), rSim.getInputVoltage()};
        inputs.velocityRadPerSec = new double[] {lSim.getAngularVelocityRadPerSec(),rSim.getAngularVelocityRadPerSec()};
        inputs.currentAmps = new double[] {lSim.getCurrentDrawAmps(), rSim.getCurrentDrawAmps()};
    }

    private double getCurrentAngle(){
        return lSim.getAngularPositionRad();
    }

    @Override
    public void setVoltage(double voltage) {
      lSim.setInputVoltage(voltage);
      rSim.setInputVoltage(voltage);
    }

        
}
