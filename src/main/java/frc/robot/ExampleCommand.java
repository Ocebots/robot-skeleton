package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class ExampleCommand extends Command {
  Drivetrain drivetrain;
  double forward;
  double turn;

  public ExampleCommand(Drivetrain drivetrain, double forward, double turn) {
    this.forward = forward;
    this.turn = turn;

    this.drivetrain = drivetrain;
    this.addRequirements(drivetrain); 
  }

  @Override
  public void execute() {
    drivetrain.drive(forward, turn);
  }

  @Override
  public void end(boolean isInterrupted) {
    drivetrain.drive(0.0, 0.0);
  }
}
