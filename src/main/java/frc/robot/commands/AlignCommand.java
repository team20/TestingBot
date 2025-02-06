package frc.robot.commands;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

/**
 * This {@code AlignCommand} supports various methods for aligning the robot.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class AlignCommand {

	/**
	 * Constructs a new {@code Command} whose purpose is to move the
	 * robot to a certain target pose.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        field-centric {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code Command} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code Command})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a new {@code Command} whose purpose is to move the
	 *         robot to a certain target pose
	 */
	public static DriveCommand driveCommand(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem,
			Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance,
			double angleTolerance) {
		// Alternative 1
		// return new DriveCommand3Controllers(driveSubsystem, () ->
		// poseEstimationSubsystem.getEstimatedPose(),
		// targetPoseSupplier, distanceTolerance, angleTolerance);

		// Alternative 2
		// return new DriveCommand(driveSubsystem, () ->
		// poseEstimationSubsystem.getEstimatedPose(),
		// targetPoseSupplier, distanceTolerance, angleTolerance);

		// Alternative 3
		// return new DriveCommand3Controllers(driveSubsystem, () ->
		// driveSubsystem.getPose(),
		// () -> {
		// Transform2d t =
		// targetPoseSupplier.get().minus(poseEstimationSubsystem.getEstimatedPose());
		// return driveSubsystem.getPose().plus(t); // odometry-centric pose of target
		// }, distanceTolerance, angleTolerance);

		// Alternative 4
		// return new DriveCommand(driveSubsystem, () -> driveSubsystem.getPose(),
		// () -> {
		// Transform2d t =
		// targetPoseSupplier.get().minus(poseEstimationSubsystem.getEstimatedPose());
		// return driveSubsystem.getPose().plus(t); // odometry-centric pose of target
		// }, distanceTolerance, angleTolerance);
		return new DriveCommand(driveSubsystem, () -> driveSubsystem.getPose(),
				() -> {
					Transform2d t = targetPoseSupplier.get().minus(poseEstimationSubsystem.getEstimatedPose());
					return driveSubsystem.getPose().plus(t); // odometry-centric pose of target
				}, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a new {@code Command} whose purpose is to move the
	 * robot to a certain target pose.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        field-centric {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code Command} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code Command})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param previous the {@code DriveCommand} right before the new
	 *        {@code Command}
	 * @return a new {@code Command} whose purpose is to move the
	 *         robot to a certain target pose
	 */
	public static DriveCommand driveCommand(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem,
			Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance,
			double angleTolerance, DriveCommand previous) {
		return new SubsequentDriveCommand(driveSubsystem, () -> driveSubsystem.getPose(),
				() -> {
					Transform2d t = targetPoseSupplier.get().minus(poseEstimationSubsystem.getEstimatedPose());
					return driveSubsystem.getPose().plus(t); // odometry-centric pose of target
				}, distanceTolerance, angleTolerance, previous);
	}

	/**
	 * Constructs a {@code Command} for following the specified path.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param intermediateToleranceRatio the ratio of distance and angle tolerances
	 *        at intermediate {@code Pose2d}s for faster movements
	 * @param poses the {@code Pose2d}s that constitute the path
	 * @return a {@code Command} for following the specified path
	 */
	public static Command follow(DriveSubsystem driveSubsystem, PoseEstimationSubsystem poseEstimationSubsystem,
			double distanceTolerance, double angleTolerance, double intermediateToleranceRatio, Pose2d[] poses) {
		List<Command> commands = new LinkedList<Command>();
		DriveCommand previous = null;
		for (var p : poses) {
			boolean last = p == poses[poses.length - 1];
			DriveCommand c = previous == null ? AlignCommand.moveTo(
					driveSubsystem, poseEstimationSubsystem, p,
					last ? distanceTolerance : intermediateToleranceRatio * distanceTolerance,
					last ? angleTolerance : intermediateToleranceRatio * angleTolerance)
					: AlignCommand.moveTo(
							driveSubsystem, poseEstimationSubsystem, p,
							last ? distanceTolerance : intermediateToleranceRatio * distanceTolerance,
							last ? angleTolerance : intermediateToleranceRatio * angleTolerance, previous);
			commands.add(c);
			previous = c;
		}
		return sequence(commands.toArray(new Command[0]));
	}

	/**
	 * Constructs a {@code Command} for moving the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPose the field-centric {@code Pose2d} of the target
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for moving the robot to the specified
	 *         target
	 */
	public static DriveCommand moveTo(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem,
			Pose2d targetPose, double distanceTolerance, double angleTolerance) {
		return moveTo(driveSubsystem, poseEstimationSubsystem, () -> targetPose, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code Command} for moving the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPose the field-centric {@code Pose2d} of the target
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param previous the {@code DriveCommand} right before the new
	 *        {@code Command}
	 * @return a {@code Commmand} for moving the robot to the specified
	 *         target
	 */
	public static DriveCommand moveTo(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem,
			Pose2d targetPose, double distanceTolerance, double angleTolerance, DriveCommand previous) {
		return moveTo(
				driveSubsystem, poseEstimationSubsystem, () -> targetPose, distanceTolerance, angleTolerance, previous);
	}

	/**
	 * Constructs a {@code Command} for moving the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        field-centric {@code Pose2d} of the target.
	 *        This is used at the commencement of the {@code Command} (i.e.,
	 *        when the scheduler begins to periodically execute the
	 *        {@code Command})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for moving the robot to the specified
	 *         target
	 */
	public static DriveCommand moveTo(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem,
			Supplier<Pose2d> targetPoseSupplier, double distanceTolerance, double angleTolerance) {
		return driveCommand(
				driveSubsystem, poseEstimationSubsystem, targetPoseSupplier,
				distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code Command} for moving the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        field-centric {@code Pose2d} of the target.
	 *        This is used at the commencement of the {@code Command} (i.e.,
	 *        when the scheduler begins to periodically execute the
	 *        {@code Command})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param previous the {@code DriveCommand} right before the new
	 *        {@code Command}
	 * @return a {@code Commmand} for moving the robot to the specified
	 *         target
	 */
	public static DriveCommand moveTo(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem,
			Supplier<Pose2d> targetPoseSupplier, double distanceTolerance, double angleTolerance,
			DriveCommand previous) {
		return driveCommand(
				driveSubsystem, poseEstimationSubsystem, targetPoseSupplier,
				distanceTolerance, angleTolerance, previous);
	}

	/**
	 * Constructs a {@code Command} for turning the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPose the field-centric {@code Pose2d} of the target
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for turning the robot to the specified
	 *         target
	 */
	public static Command turnTo(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem,
			Pose2d targetPose, double distanceTolerance, double angleTolerance) {
		return turnTo(driveSubsystem, poseEstimationSubsystem, () -> targetPose, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code Command} for turning the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        field-centric {@code Pose2d} of the target.
	 *        This is used at the commencement of the {@code Command} (i.e.,
	 *        when the scheduler begins to periodically execute the
	 *        {@code Command})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for turning the robot to the specified
	 *         target
	 */
	public static Command turnTo(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem,
			Supplier<Pose2d> targetPoseSupplier, double distanceTolerance, double angleTolerance) {
		return driveCommand(
				driveSubsystem, poseEstimationSubsystem,
				() -> {
					Transform2d t = new Transform2d(0, 0,
							poseEstimationSubsystem.angularDisplacement(targetPoseSupplier.get()));
					return poseEstimationSubsystem.getEstimatedPose().plus(t);
				},
				distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code Command} for moving the robot toward the specified
	 * target position while ensuring that the robot is away from the target by the
	 * specified distance.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPosition the field-centric {@code Translation2d} (i.e., the
	 *        position) of the
	 *        target
	 * @param distanceToTarget the desired distance between the robot and the
	 *        target position
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for turning the robot to the specified target
	 *         position
	 */
	public static Command moveToward(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem, Translation2d targetPosition, double distanceToTarget,
			double distanceTolerance, double angleTolerance) {
		return moveToward(
				driveSubsystem, poseEstimationSubsystem, () -> targetPosition, distanceToTarget, distanceTolerance,
				angleTolerance);
	}

	/**
	 * Constructs a {@code Command} for moving the robot toward the specified
	 * target position while ensuring that the robot is away from the target by the
	 * specified distance.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPositionSupplier a {@code Supplier<Pose2d>} that provides the
	 *        field-centric {@code Translation2d} (i.e., the position) of the
	 *        target.
	 *        This is used at the commencement of the {@code Command} (i.e.,
	 *        when the scheduler begins to periodically execute the
	 *        {@code Command})
	 * @param distanceToTarget the desired distance between the robot and the
	 *        target position
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for turning the robot to the specified target
	 *         position
	 */
	public static Command moveToward(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem, Supplier<Translation2d> targetPositionSupplier,
			double distanceToTarget, double distanceTolerance, double angleTolerance) {
		return driveCommand(
				driveSubsystem, poseEstimationSubsystem,
				() -> {
					Transform2d t = poseEstimationSubsystem
							.transformationToward(targetPositionSupplier.get(), distanceToTarget);
					return poseEstimationSubsystem.getEstimatedPose().plus(t);
				},
				distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code Command} for turning the robot to the closest
	 * {@code AprilTag} on the field.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param angleOfCoverageInDegrees the angular coverage (in degrees) within
	 *        which {@code AprilTag}s are considered (maximum: 180)
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for turning the robot to the specified
	 *         target
	 */
	public static Command turnToClosestTag(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem, double angleOfCoverageInDegrees,
			double distanceThresholdInMeters, double distanceTolerance, double angleTolerance) {
		return turnTo(
				driveSubsystem, poseEstimationSubsystem,
				() -> poseEstimationSubsystem.closestTagPose(angleOfCoverageInDegrees, distanceThresholdInMeters),
				distanceThresholdInMeters, angleTolerance);
	}

	/**
	 * Constructs a {@code Command} for moving the robot to the closest
	 * {@code AprilTag} on the field.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param angleOfCoverageInDegrees the angular coverage (in degrees) within
	 *        which {@code AprilTag}s are considered (maximum: 180)
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @param robotToTarget the {@code Pose2d} of the closest {@code AprilTag}
	 *        relative to the {@code Pose2d} of the robot for the {@code Command} to
	 *        finish
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for moving the robot to the specified
	 *         target
	 */
	public static Command moveToClosestTag(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem, double angleOfCoverageInDegrees,
			double distanceThresholdInMeters, Transform2d robotToTarget, double distanceTolerance,
			double angleTolerance) {
		return moveTo(
				driveSubsystem, poseEstimationSubsystem,
				() -> poseEstimationSubsystem.closestTagPose(angleOfCoverageInDegrees, distanceThresholdInMeters)
						.plus(robotToTarget),
				distanceTolerance, angleTolerance);
	}

}