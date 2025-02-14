package frc.robot;

import static edu.wpi.first.math.util.Units.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.subsystems.PoseEstimationSubsystem.*;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.PathDriveCommand;
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
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance,
						pose(feetToMeters(distanceInFeet), 0, 0)),
				Commands.waitSeconds(2),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)),
				Commands.waitSeconds(1),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)));
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
				new DriveCommand(m_driveSubsystem,
						distanceTolerance, angleTolerance, pose(0.0, 0, 0)).withTimeout(1),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(sideLength, 0, 90))
						.withTimeout(timeout / 4),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(sideLength, sideLength, 180))
						.withTimeout(timeout / 4),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, sideLength, 270))
						.withTimeout(timeout / 4),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0.0, 0))
						.withTimeout(timeout / 4),
				new DriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, pose(0.0, 0, 0)));
	}

	/**
	 * Returns a {@code Command} for moving the robot on a circle.
	 * 
	 * @param radius the radius of the circle in meters
	 * @param initialAngularIncrement the initial angular increment in degrees which
	 *        describes how quickly the robot to move on the circle
	 * @param finalAngularIncrement the final angular increment in degrees which
	 *        describes how quickly the robot to move on the circle
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param intermediateToleranceRatio the ratio to apply to the distance and
	 *        angle tolerances for intermeidate target {@code Pose2d}s
	 * @param poseCount the number of {@code Pose2d}s on the circle
	 * 
	 * @return a {@code Command} for moving the robot on a circle
	 */
	public static Command moveOnOval(double radius, double initialAngularIncrement, double finalAngularIncrement,
			double distanceTolerance, double angleTolerance, double intermediateToleranceRatio, int poseCount) {
		Rotation2d angle = Rotation2d.kZero;
		var l = new LinkedList<Pose2d>();
		for (double i = 0; i < poseCount; i++) {
			var t = translation(radius, 0).rotateBy(angle);
			l.add(new Pose2d(translation(t.getX() + radius, t.getY() / 2), angle));
			double progress = i / poseCount;
			double angularVelocity = progress * finalAngularIncrement + (1 - progress) * initialAngularIncrement;
			angle = angle.plus(rotation(angularVelocity));
		}
		return new PathDriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, intermediateToleranceRatio,
				l.toArray(new Pose2d[0]));
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

	/**
	 * Returns a {@code Command} for aligning the robot to the specified
	 * {@code AprilTag}.
	 * 
	 * @param tagID the ID of the {@code AprilTag}
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param intermediateToleranceRatio the ratio to apply to the distance and
	 *        angle tolerances for intermeidate target {@code Pose2d}s
	 * @param robotToTag the {@code Pose2d}s of the {@code AprilTag} relative to the
	 *        center of the robot when the alignment is completed
	 * @param robotToTagReady the {@code Pose2d}s of the {@code AprilTag} relative
	 *        to the center of the robot when the alignment ready step is completed
	 * 
	 * @return a {@code Command} for aligning the robot to the specified
	 *         {@code AprilTag}
	 */
	public static Command alignToTag(int tagID, double distanceTolerance, double angleTolerance,
			double intermediateToleranceRatio, Transform2d robotToTag, Transform2d robotToTagReady) {
		return new PathDriveCommand(m_driveSubsystem, distanceTolerance, angleTolerance, intermediateToleranceRatio,
				List.of(
						() -> m_poseEstimationSubsystem
								.odometryCentricPose(kFieldLayout.getTagPose(tagID).get().toPose2d())
								.plus(robotToTagReady),
						() -> m_poseEstimationSubsystem
								.odometryCentricPose(kFieldLayout.getTagPose(tagID).get().toPose2d())
								.plus(robotToTag)));
	}

	/**
	 * Returns a {@code Command} for aligning the robot to the specified
	 * {@code AprilTag}s.
	 * 
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param intermediateToleranceRatio the ratio to apply to the distance and
	 *        angle tolerances for intermeidate target {@code Pose2d}s
	 * @param robotToTag the {@code Pose2d}s of the {@code AprilTag} relative to the
	 *        center of the robot when the alignment is completed
	 * @param robotToTagReady the {@code Pose2d}s of the {@code AprilTag} relative
	 *        to the center of the robot when the alignment ready step is completed
	 * @param tagID the ID of the {@code AprilTag}
	 * 
	 * @return a {@code Command} for aligning the robot to the specified
	 *         {@code AprilTag}s
	 */
	public static Command alignToTags(double distanceTolerance, double angleTolerance,
			double intermediateToleranceRatio, Transform2d robotToTag, Transform2d robotToTagReady, int... tagIDs) {
		List<Command> l = Arrays.stream(tagIDs).mapToObj(
				i -> alignToTag(
						i, distanceTolerance, angleTolerance, intermediateToleranceRatio, robotToTag, robotToTagReady))
				.toList();
		return sequence(l.toArray(new Command[0]));
	}

}
