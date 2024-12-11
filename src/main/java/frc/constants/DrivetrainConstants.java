package frc.constants;

import com.revrobotics.CANSparkBase.IdleMode;

public class DrivetrainConstants {
  public static final int CURRENT_LIMIT = 20;
  public static final IdleMode IDLE_MODE = IdleMode.kBrake;

  public static final double WHEEL_RADIUS = 0.1524 / 2.0; // meters
  public static final double GEAR_RATIO = 10.71;

  public static final double POSITION_CONVERSION_FACTOR =
      1.0 / GEAR_RATIO * 2.0 * Math.PI * WHEEL_RADIUS; // rotations to meters
  public static final double VELOCITY_CONVERSION_FACTOR =
      POSITION_CONVERSION_FACTOR / 60; // rotations per min to meters per sec

  public static final double MODULE_P = 0.04;
  public static final double MODULE_I = 0;
  public static final double MODULE_D = 0;
  public static final double MODULE_S = 0;
  public static final double MODULE_V = 2.74;
  public static final double MODULE_A = 0.10;

  public static final double TRACK_WIDTH = 0.5842; // meters

  public static final int FRONT_LEFT_MOTOR_ID = 2;
  public static final int FRONT_RIGHT_MOTOR_ID = 4;
  public static final int REAR_LEFT_MOTOR_ID = 1;
  public static final int REAR_RIGHT_MOTOR_ID = 3;

  public static final double MAX_VELOCITY = 3.75; // meters per second

  public static final double TRAJECTORY_MAX_VELOCITY = 1.0;
  public static final double TRAJECTORY_MAX_ACCEL = 1.0;
}
