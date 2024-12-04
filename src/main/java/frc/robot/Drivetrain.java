package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax left_motor = new CANSparkMax(2, MotorType.kBrushless); 
  private CANSparkMax right_motor = new CANSparkMax(4, MotorType.kBrushless); 
  private CANSparkMax left_rear_motor = new CANSparkMax(1, MotorType.kBrushless); 
  private CANSparkMax right_rear_motor = new CANSparkMax(3, MotorType.kBrushless); 

  private DifferentialDrive drive = new DifferentialDrive(left_motor, right_motor);

  public static void configureMotor(CANSparkMax motor, boolean reversed) {
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(10);
    motor.setInverted(reversed);
    motor.setIdleMode(IdleMode.kBrake);
  }

  public Drivetrain() {
    drive.setSafetyEnabled(false);

    configureMotor(left_motor, true);
    configureMotor(left_rear_motor, true);
    configureMotor(right_motor, false);
    configureMotor(right_rear_motor, false);

    left_rear_motor.follow(left_motor);
    right_rear_motor.follow(right_motor);
  }

  public void drive(double forward, double rotate) {
    drive.arcadeDrive(forward, rotate); 
  }

  private void stop() {
    drive.arcadeDrive(0.0, 0.0);
  }

  public Command driveCommand(double forward, double rotate) {
    return Commands.runEnd(() -> {
      drive.arcadeDrive(forward, rotate);
    }, () -> {
      this.stop();
    });
  }
}
