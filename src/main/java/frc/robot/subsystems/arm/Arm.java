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
    private double voltageCMDFF = 0.0;

    private final ProfiledPIDController PIDcontroller = new ProfiledPIDController(Constants.ArmConstants.ARM_P, Constants.ArmConstants.ARM_I, Constants. ArmConstants.ARM_D, 
    new TrapezoidProfile.Constraints(Constants.ArmConstants.ARM_MAX_VEL, Constants.ArmConstants.ARM_MAX_ACCEL));

    private static final LoggedTunableNumber armKg = new LoggedTunableNumber("Arm/kG");
    private static final LoggedTunableNumber armKv = new LoggedTunableNumber("Arm/kV");
    private static final LoggedTunableNumber armKa = new LoggedTunableNumber("Arm/kA");

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
        armMaxAccelerationRad.initDefault(Constants.ArmConstants.ARM_MAX_ACCEL);
        armMaxVelocityRad.initDefault(Constants.ArmConstants.ARM_MAX_VEL);
    }


    @Override
    public void periodic(){

        io.updateInputs(inputs);

        voltageCMDFF = armFF.calculate(inputs.angleRads, Constants.ArmConstants.ARM_MAX_VEL * -1
        //manual);
        );
        switch(state){
            case MANUAL:
            break;
            case POS:
            break;
        } 
    }

    public void setTargetangle(){
        
    }


}
