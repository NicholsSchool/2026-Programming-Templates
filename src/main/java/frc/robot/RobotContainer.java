package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public RobotContainer() {
   
    }



  private void initShuffleboard() {
   
  }

  public void updateShuffleboard() {
   
  }



 
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  // /**
  //  * @return the command to run in autonomous
  //  */
  public Command getAutonomousCommand() {


     try{
        // Load the path you want to follow using its name in the GUI
        // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
        // System.out.println("ACTIVE PATH CALL BACK 3");
        // return AutoBuilder.followPath(path);
          return new InstantCommand();
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }

    //   // var path = PathPlannerAuto.getStaringPoseFromAutoFile("TestAuto");
    //   // var path = PathPlannerPath.fromChoreoTrajectory("New Path");
    //   // NamedCommands.registerCommand("RunIntake", intake.runEatCommand().withTimeout(1.0));
    //   // NamedCommands.registerCommand("StopIntake", new InstantCommand(() -> intake.stop(),
    // intake));
    //   // NamedCommands.registerCommand(
    //   //     "Shoot", new InstantCommand(() -> new Shoot(shooter, intake,
    // indexer)).withTimeout(3));
    //   // drive.setPose(PathPlannerAuto.getStaringPoseFromAutoFile("New Auto"));
    //   // return new PathPlannerAuto("New Auto");
    //   // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("TestPath"));
    //registerNamedCommands();
    //return autoChooser.get();
  }

  private void addAutos() {}

}
