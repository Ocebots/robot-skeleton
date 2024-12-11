package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.constants.DrivetrainConstants;
import java.util.List;
import java.util.Set;

public class Drivetrain extends SubsystemBase {
  // The motors and their sensors
  private final TankModule leftModule =
      new TankModule(
          DrivetrainConstants.FRONT_LEFT_MOTOR_ID, DrivetrainConstants.REAR_LEFT_MOTOR_ID, false);
  private final TankModule rightModule =
      new TankModule(
          DrivetrainConstants.FRONT_RIGHT_MOTOR_ID, DrivetrainConstants.REAR_RIGHT_MOTOR_ID, true);

  // The 9-axis inertial measurement unit (measures our angle)
  private final AHRS gyro = new AHRS();

  // This does the math that converts our desired robot speed (ChassisSpeeds) to actual wheel
  // speeds
  private final DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(DrivetrainConstants.TRACK_WIDTH);

  // This does the math that converts our current angle and the change in position of the wheels
  // into the global robot position
  private final DifferentialDrivePoseEstimator drivePoseEstimator =
      new DifferentialDrivePoseEstimator(
          driveKinematics,
          getHeading(),
          leftModule.getPosition(),
          rightModule.getPosition(),
          new Pose2d());

  // Controls how fast the robot moves and accelerates while following trajectories (used for
  // driving a distance or turning)
  private final TrajectoryConfig trajectoryConfig =
      new TrajectoryConfig(
              DrivetrainConstants.TRAJECTORY_MAX_VELOCITY, DrivetrainConstants.TRAJECTORY_MAX_ACCEL)
          .addConstraint(
              new DifferentialDriveKinematicsConstraint(
                  driveKinematics, DrivetrainConstants.MAX_VELOCITY));

  // Used to display robot position
  private final Field2d field = new Field2d();

  public Drivetrain() {
    // Add the field onto the dashboard
    SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    // Update our position estimate
    drivePoseEstimator.update(getHeading(), leftModule.getPosition(), rightModule.getPosition());
    // Update the display
    field.setRobotPose(getCurrentPose());
  }

  /**
   * @return the heading of the robot relative to where it started, ccw positive
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(gyro.getYaw() - 360);
  }

  public Pose2d getCurrentPose() {
    return drivePoseEstimator.getEstimatedPosition();
  }

  /**
   * Set the desired speeds of the robot and update the wheels control loops. This should be called
   * repeatedly if you want the drivetrain to continue moving.
   */
  public void setSpeeds(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = driveKinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(DrivetrainConstants.MAX_VELOCITY);
    leftModule.setVelocity(wheelSpeeds.leftMetersPerSecond);
    rightModule.setVelocity(wheelSpeeds.rightMetersPerSecond);
  }

  private Command followTrajectory(Trajectory trajectory) {
    return new RamseteCommand(
        trajectory,
        this::getCurrentPose,
        new RamseteController(),
        driveKinematics,
        (Double leftSpeed, Double rightSpeed) -> {
          leftModule.setVelocity(leftSpeed);
          rightModule.setVelocity(rightSpeed);
        },
        this);
  }

  /** Move the robot along a given transform from the pose it started at. */
  private Command driveTransform(Transform2d transform) {
    return Commands.defer(
        () -> {
          Pose2d currentPose = getCurrentPose();
          Trajectory trajectory =
              TrajectoryGenerator.generateTrajectory(
                  currentPose, List.of(), currentPose.transformBy(transform), trajectoryConfig);

          return followTrajectory(trajectory);
        },
        Set.of(this));
  }

  /** Drive a distance in meters in the current direction the robot is facing */
  public Command driveForward(double distance) {
    return driveTransform(new Transform2d(distance, 0, new Rotation2d(0)));
  }

  /** Turn right (cw) 90 degrees */
  public Command turnRight() {
    return driveTransform(new Transform2d(0, 0, Rotation2d.fromDegrees(-90)));
  }

  /** Turn left (ccw) 90 degrees */
  public Command turnLeft() {
    return driveTransform(new Transform2d(0, 0, Rotation2d.fromDegrees(90)));
  }

  /** Stop the robot from moving */
  public void stop() {
    leftModule.stop();
    rightModule.stop();
  }
}
