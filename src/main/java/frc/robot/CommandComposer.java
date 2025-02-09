package frc.robot;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import edu.wpi.first.math.geometry.Rotation2d;
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
						distanceTolerance, angleTolerance).withDeadline(Commands.waitSeconds(1)),
				new DriveCommand2Controllers(m_driveSubsystem, pose(feetToMeters(distanceInFeet), 0, 0),
						distanceTolerance, angleTolerance),
				Commands.waitSeconds(2),
				new DriveCommand2Controllers(m_driveSubsystem, pose(0.0, 0, 0),
						distanceTolerance, angleTolerance),
				Commands.waitSeconds(2),
				new DriveCommand2Controllers(m_driveSubsystem, pose(0.0, 0, 0),
						distanceTolerance, angleTolerance));
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
						0, 0, a.getDegrees() * kTurnP,
						false);
		}).withTimeout(1);
	}

}
