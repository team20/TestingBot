package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.*;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AlignCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveCommandOld;
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
	 * Returns a {@code Command} for testing subsystems and {@code Command}s.
	 * 
	 * @return a {@code Command} for testing subsystems and {@code Command}s
	 */
	public static Command testSubsystemsAndCommands() {
		return sequence(
				m_driveSubsystem.testCommand(), // F, B, SL, SR, RL, RR
				DriveCommandOld.testCommand(m_driveSubsystem).withTimeout(2),
				DriveCommand.testCommand(m_driveSubsystem).withTimeout(2));
	}

	/**
	 * Returns a {@code Command} for moving forward and then backward.
	 * 
	 * @param distanceInFeet the distance in feet
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * 
	 * @return a {@code Command} for moving 6 feet forward and then backward.
	 */
	public static Command moveForwardBackward(double distanceInFeet, double distanceTolerance,
			double angleTolerance) {
		return sequence(
				DriveCommand
						.moveForward(m_driveSubsystem, 0.0254 * 12 * distanceInFeet, distanceTolerance, angleTolerance),
				DriveCommand
						.moveForward(
								m_driveSubsystem, -0.0254 * 12 * distanceInFeet, distanceTolerance, angleTolerance));
	}

	/**
	 * Returns a {@code Command} for moving forward and then backward using 3 PID
	 * controllers.
	 * 
	 * @param distanceInFeet the distance in feet
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * 
	 * @return a {@code Command} for moving forward and then backward using 3 PID
	 *         controllers.
	 */
	public static Command moveForwardBackward3Controllers(double distanceInFeet, double distanceTolerance,
			double angleTolerance) {
		return sequence(
				DriveCommandOld
						.moveForward(m_driveSubsystem, 0.0254 * 12 * distanceInFeet, distanceTolerance, angleTolerance),
				DriveCommandOld
						.moveForward(
								m_driveSubsystem, -0.0254 * 12 * distanceInFeet, distanceTolerance, angleTolerance));
	}

	/**
	 * Returns a {@code Command} for visiting all the specified {@code AprilTag}s.
	 * 
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param robotToTag the {@code Tranform2d} representing the pose of the
	 *        {@code AprilTag} relative to the robot when the robot is aligned
	 * @param tagIDs the IDs of the {@code AprilTag}s
	 * @return a {@code Command} for visiting all the specified {@code AprilTag}s
	 */
	public static Command visitTags(double distanceTolerance, double angleTolerance, Transform2d robotToTag,
			int... tagIDs) {
		List<DriveCommand> commands = Arrays.stream(tagIDs).mapToObj(i -> kFieldLayout.getTagPose(i))
				.filter(p -> p.isPresent())
				.map(p -> p.get())
				.map(p -> p.toPose2d().plus(robotToTag)).map(
						p -> AlignCommand
								.moveTo(
										m_driveSubsystem, m_poseEstimationSubsystem, p, distanceTolerance,
										angleTolerance))
				.toList();
		return sequence(commands.toArray(new Command[0]));
	}

	/**
	 * Returns a {@code Command} for visiting all the specified {@code AprilTag}s.
	 * 
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param robotToTag the {@code Tranform2d} representing the pose of the
	 *        {@code AprilTag} relative to the robot when the robot is aligned
	 * @param tagIDs the IDs of the {@code AprilTag}s
	 * @return a {@code Command} for visiting all the specified {@code AprilTag}s
	 */
	public static Command visitTagsOptimized(double distanceTolerance, double angleTolerance, Transform2d robotToTarget,
			int... tagIDs) {
		var l = Arrays.stream(tagIDs).mapToObj(i -> kFieldLayout.getTagPose(i))
				.filter(p -> p.isPresent())
				.map(p -> p.get())
				.map(p -> p.toPose2d().plus(robotToTarget)).toList();
		List<Command> commands = new LinkedList<Command>();
		DriveCommand previous = null;
		for (var p : l) {
			boolean last = p == l.get(l.size() - 1);
			DriveCommand c = previous == null ? AlignCommand.moveTo(
					m_driveSubsystem, m_poseEstimationSubsystem, p, last ? distanceTolerance : 3 * distanceTolerance,
					last ? angleTolerance : 3 * angleTolerance)
					: AlignCommand.moveTo(
							m_driveSubsystem, m_poseEstimationSubsystem, p,
							last ? distanceTolerance : 3 * distanceTolerance,
							last ? angleTolerance : 3 * angleTolerance, previous);
			commands.add(c);
			previous = c;
		}
		return sequence(commands.toArray(new Command[0]));
	}

	/**
	 * Returns a {@code Command} for moving to the specified {@code AprilTag} in two
	 * steps.
	 * 
	 * @param tagID the ID of the {@code AprilTag}
	 * 
	 * @param robotToTagReady the {@code Tranform2d} representing the pose of the
	 *        {@code AprilTag} relative to the robot when the robot is tentatively
	 *        aligned (1st step)
	 * @param robotToTag the {@code Tranform2d} representing the pose of the
	 *        {@code AprilTag} relative to the robot when the robot is fully aligned
	 *        (2nd step)
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Command} for moving to the specified {@code AprilTag} in two
	 *         steps.
	 */
	public static Command moveToTag(int tagID, Transform2d robotToTagReady, Transform2d robotToTag,
			double distanceTolerance,
			double angleTolerance) {
		DriveCommand c1 = AlignCommand.moveTo(
				m_driveSubsystem, m_poseEstimationSubsystem,
				kFieldLayout.getTagPose(tagID).get().toPose2d().plus(robotToTagReady),
				distanceTolerance, angleTolerance);
		DriveCommand c2 = AlignCommand.moveTo(
				m_driveSubsystem, m_poseEstimationSubsystem,
				kFieldLayout.getTagPose(tagID).get().toPose2d().plus(robotToTag),
				distanceTolerance, angleTolerance, c1);
		return sequence(
				c1, c2, AlignCommand.moveTo(
						m_driveSubsystem, m_poseEstimationSubsystem,
						kFieldLayout.getTagPose(tagID).get().toPose2d().plus(robotToTagReady),
						distanceTolerance, angleTolerance, c2));
	}

	/**
	 * Returns a {@code Command} for visiting all the specified {@code AprilTag}s.
	 * 
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param robotToTagReady the {@code Tranform2d} representing the pose of the
	 *        {@code AprilTag} relative to the robot when the robot is tentatively
	 *        aligned (1st step)
	 * @param robotToTag the {@code Tranform2d} representing the pose of the
	 *        {@code AprilTag} relative to the robot when the robot is fully aligned
	 *        (2nd step)
	 * @param tagIDs the IDs of the {@code AprilTag}s
	 * @return a {@code Command} for visiting all the specified {@code AprilTag}s
	 */
	public static Command visitTags(double distanceTolerance, double angleTolerance, Transform2d robotToTagReady,
			Transform2d robotToTag,
			int... tagIDs) {
		List<Command> commands = Arrays.stream(tagIDs)
				.mapToObj(i -> moveToTag(i, robotToTagReady, robotToTag, distanceTolerance, angleTolerance))
				.toList();
		return sequence(commands.toArray(new Command[0]));
	}

}
