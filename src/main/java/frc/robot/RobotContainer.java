package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

/**
 * This is replaces {@link Robot} as the class where we put all of our code. This has different
 * methods because its {@link Command} based not time based.
 */
public class RobotContainer {
  Drivetrain drivetrain = new Drivetrain();
  CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    // If the drivetrain is not doing something else, listen to user input
    drivetrain.setDefaultCommand(
        Commands.runEnd(
            () ->
                drivetrain.setSpeeds(
                    new ChassisSpeeds(-controller.getLeftY(), 0, -controller.getRightX())),
            () -> drivetrain.stop(),
            drivetrain));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous routine yet");
  }
}
