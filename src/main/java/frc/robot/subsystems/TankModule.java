package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.constants.DrivetrainConstants;

public class TankModule {
  private final CANSparkMax motor;
  private final CANSparkMax follower_motor;
  private final RelativeEncoder encoder;

  // Controllers to maintain a given velocity
  private final PIDController controller =
      new PIDController(
          DrivetrainConstants.MODULE_P, DrivetrainConstants.MODULE_I, DrivetrainConstants.MODULE_D);

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          DrivetrainConstants.MODULE_S, DrivetrainConstants.MODULE_V, DrivetrainConstants.MODULE_A);

  public TankModule(int frontId, int backId, boolean reversed) {
    // configure motors
    motor = new CANSparkMax(frontId, MotorType.kBrushless);
    follower_motor = new CANSparkMax(backId, MotorType.kBrushless);
    configureMotor(motor, reversed);
    configureMotor(follower_motor, reversed);
    follower_motor.follow(motor);

    // configure encoder
    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(DrivetrainConstants.POSITION_CONVERSION_FACTOR);
    encoder.setVelocityConversionFactor(DrivetrainConstants.VELOCITY_CONVERSION_FACTOR);
  }

  /**
   * @return the velocity in meters per second
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * @return the position in meters
   */
  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * This should be called repeatedly
   *
   * @param targetVelocity in meters per second
   */
  public void setVelocity(double targetVelocity) {
    double currentVelocity = getVelocity();
    motor.setVoltage(
        controller.calculate(currentVelocity, targetVelocity)
            + feedforward.calculate(targetVelocity));
  }

  /** Stop both motors */
  public void stop() {
    motor.stopMotor();
  }

  private static void configureMotor(CANSparkMax motor, boolean reversed) {
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(DrivetrainConstants.CURRENT_LIMIT);
    motor.setInverted(reversed);
    motor.setIdleMode(DrivetrainConstants.IDLE_MODE);
  }
}
