package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Drivetrain {
  private Spark left_motor = new Spark(0); 
  private Spark right_motor = new Spark(1); 

  private DifferentialDrive drive = new DifferentialDrive(left_motor, right_motor);

  public Drivetrain() {
    right_motor.setInverted(true);
  }

  public void drive(double forward, double rotate) {
    drive.arcadeDrive(forward * .5, -rotate * .5); 
  }
}
