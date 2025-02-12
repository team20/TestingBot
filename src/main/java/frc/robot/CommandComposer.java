package frc.robot;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.drive.DriveCommand2Controllers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class CommandComposer {
	private static DriveSubsystem m_driveSubsystem;
	private static PoseEstimationSubsystem m_poseEstimationSubsystem;

	public static void setSubsystems(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_poseEstimationSubsystem = poseEstimationSubsystem;
	}

	/**
	 * Creates a {@code Command} for testing the {@code DriveSubsystem}. The robot
	 * must move forward and then backward while rotating left and then right
	 * relative to the field.
	 * 
	 * @return a {@code Command} for testing the {@code DriveSubsystem}. The robot
	 *         must move forward and then backward while rotating left and then
	 *         right relative to the field.
	 */
	public static Command testDriveSubsystemFieldRelative() {
		double speed = 1;
		double rotionalSpeed = Math.toRadians(45);
		double duration = 2.0;
		return sequence(
				m_driveSubsystem.run(() -> m_driveSubsystem.setModuleAngles(0)).withTimeout(.1),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(speed, 0, rotionalSpeed, true)).withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(-speed, 0, -rotionalSpeed, true))
						.withTimeout(duration));
	}

	/**
	 * Returns a {@code Command} for testing all subsystems.
	 * 
	 * @return a {@code Command} for testing all subsystems
	 */
	public static Command testAllSubsystems() {
		return sequence(
				m_driveSubsystem.testCommand());
	}

	/**
	 * Returns a {@code Command} for testing the rotation capability of the robot.
	 * 
	 * @return a {@code Command} for testing the rotation capability of the robot
	 */
	public static Command testRotation() {
		double rotionalSpeed = kTurnMaxAngularSpeed * 0.9;
		double duration = 2.0;
		return sequence(
				m_driveSubsystem.run(() -> m_driveSubsystem.setModuleAngles(0)).withTimeout(0.1),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(.5, 0, rotionalSpeed, true))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(-.5, 0, -rotionalSpeed, true))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0.05, 0, 0, false))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0, 0, rotionalSpeed, false))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0.05, 0, 0, false))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0, 0, -rotionalSpeed, false))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0.05, 0, 0, false))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0, 0, rotionalSpeed, false))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0.05, 0, 0, false))
						.withTimeout(duration),
				m_driveSubsystem.run(() -> m_driveSubsystem.drive(0, 0, -rotionalSpeed, false))
						.withTimeout(duration));
	}

	/**
	 * Returns a {@code Command} for moving forward and then backward.
	 * 
	 * @param distanceInFeet the distance in feet
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * 
	 * @return a {@code Command} for moving forward and then backward.
	 */
	public static Command moveForwardBackward2Controllers(double distanceInFeet, double distanceTolerance,
			double angleTolerance) {
		return sequence(
				new DriveCommand2Controllers(m_driveSubsystem, pose(0.0, 0, 0),
						distanceTolerance, angleTolerance),
				new DriveCommand2Controllers(m_driveSubsystem, pose(feetToMeters(distanceInFeet), 0, 0),
						distanceTolerance, angleTolerance),
				Commands.waitSeconds(2),
				new DriveCommand2Controllers(m_driveSubsystem, pose(0.0, 0, 0),
						distanceTolerance, angleTolerance),
				Commands.waitSeconds(1),
				new DriveCommand2Controllers(m_driveSubsystem, pose(0.0, 0, 0),
						distanceTolerance, angleTolerance));
	}

	/**
	 * Returns a {@code Command} for moving the robot on a square.
	 * 
	 * @param sideLength the side length of the square in meters
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param timeout the maximum amount of the time given to the {@code Command}
	 * 
	 * @return a {@code Command} for moving the robot on a circle
	 */
	public static Command moveOnSquare(double sideLength, double distanceTolerance,
			double angleTolerance, double timeout) {
		return sequence(
				new DriveCommand2Controllers(m_driveSubsystem, pose(0.0, 0, 0),
						distanceTolerance, angleTolerance).withTimeout(1),
				new DriveCommand2Controllers(m_driveSubsystem, pose(sideLength, 0, 90),
						distanceTolerance, angleTolerance).withTimeout(timeout / 4),
				new DriveCommand2Controllers(m_driveSubsystem, pose(sideLength, sideLength, 180),
						distanceTolerance, angleTolerance).withTimeout(timeout / 4),
				new DriveCommand2Controllers(m_driveSubsystem, pose(0.0, sideLength, 270),
						distanceTolerance, angleTolerance).withTimeout(timeout / 4),
				new DriveCommand2Controllers(m_driveSubsystem, pose(0.0, 0.0, 0),
						distanceTolerance, angleTolerance).withTimeout(timeout / 4),
				new DriveCommand2Controllers(m_driveSubsystem, pose(0.0, 0, 0),
						distanceTolerance, angleTolerance));
	}

	/**
	 * Returns a {@code Command} for moving the robot on a circle.
	 * 
	 * @param radius the radius of the circle in meters
	 * @param initialAngularVelocity the initial angular velocity in degrees per
	 *        second which describes how quickly the robot is moving on the circle
	 * @param finalAngularVelocity the final angular velocity in degrees per
	 *        second which describes how quickly the robot is moving on the circle
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param timeout the maximum amount of the time given to the {@code Command}
	 * 
	 * @return a {@code Command} for moving the robot on a circle
	 */
	public static Command moveOnCircle(double radius, double initialAngularVelocity, double finalAngularVelocity,
			double distanceTolerance, double angleTolerance, double timeout) {

		Timer timer = new Timer();

		Supplier<Pose2d> s = new Supplier<Pose2d>() {

			Rotation2d angle = Rotation2d.kZero;

			@Override
			public Pose2d get() {
				var p = new Pose2d(translation(radius, 0).rotateBy(angle), angle);
				double progress = timer.get() / timeout;
				double angularVelocity = progress * finalAngularVelocity + (1 - progress) * initialAngularVelocity;
				angle = angle.plus(rotation(angularVelocity));
				return p;
			}
		};
		return new DriveCommand2Controllers(m_driveSubsystem, s, true, distanceTolerance, angleTolerance) {

			@Override
			public void initialize() {
				super.initialize();
				timer.restart();
			}

		}.withTimeout(timeout);
	}

	/**
	 * Returns a {@code Command} for turning the robot toward the specified
	 * {@code AprilTag}.
	 * 
	 * @param tagID the ID of the {@code AprilTag}
	 * @return a {@code Command} for turning the robot toward the specified
	 *         {@code AprilTag}
	 */
	public static Command turnTowardTag(int tagID) {
		return m_driveSubsystem.run(() -> {
			Rotation2d a = m_poseEstimationSubsystem.angularDisplacement(1);
			if (a != null)
				m_driveSubsystem.drive(
						0, 0, a.getRadians() * kTurnP,
						false);
		}).withTimeout(1);
	}

}
