package frc.robot.commands;

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
public class AlignCommand extends Command {

	/**
	 * Constructs a {@code DriveCommand} for moving the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPose the {@code Pose2d} of the target
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for moving the robot to the specified
	 *         target
	 */
	public static DriveCommand moveTo(DriveSubsystem driveSubsystem, PoseEstimationSubsystem poseEstimationSubsystem,
			Pose2d targetPose, double distanceTolerance, double angleTolerance) {
		return moveTo(driveSubsystem, poseEstimationSubsystem, () -> targetPose, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code DriveCommand} for moving the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} of the target.
	 *        This is used at the commencement of the {@code DriveCommand} (i.e.,
	 *        when the scheduler begins to periodically execute the
	 *        {@code DriveCommand})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for moving the robot to the specified
	 *         target
	 */
	public static DriveCommand moveTo(DriveSubsystem driveSubsystem, PoseEstimationSubsystem poseEstimationSubsystem,
			Supplier<Pose2d> targetPoseSupplier, double distanceTolerance, double angleTolerance) {
		return new DriveCommand(driveSubsystem, () -> poseEstimationSubsystem.getEstimatedPose(),
				targetPoseSupplier,
				distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code DriveCommand} for turning the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPose the {@code Pose2d} of the target
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for turning the robot to the specified
	 *         target
	 */
	public static DriveCommand turnTo(DriveSubsystem driveSubsystem, PoseEstimationSubsystem poseEstimationSubsystem,
			Pose2d targetPose, double distanceTolerance, double angleTolerance) {
		return turnTo(driveSubsystem, poseEstimationSubsystem, () -> targetPose, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code DriveCommand} for turning the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} of the target.
	 *        This is used at the commencement of the {@code DriveCommand} (i.e.,
	 *        when the scheduler begins to periodically execute the
	 *        {@code DriveCommand})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for turning the robot to the specified
	 *         target
	 */
	public static DriveCommand turnTo(DriveSubsystem driveSubsystem, PoseEstimationSubsystem poseEstimationSubsystem,
			Supplier<Pose2d> targetPoseSupplier, double distanceTolerance, double angleTolerance) {
		return new DriveCommand(driveSubsystem, () -> poseEstimationSubsystem.getEstimatedPose(),
				() -> {
					var transform = new Transform2d(0, 0,
							poseEstimationSubsystem.angularDisplacement(targetPoseSupplier.get()));
					return poseEstimationSubsystem.getEstimatedPose().plus(transform);
				},
				distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code DriveCommand} for moving the robot toward the specified
	 * target position while ensuring that the robot is away from the target by the
	 * specified distance.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimaitonSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPosition a {@code Translation2d} (i.e., the position) of the
	 *        target.
	 * @param distanceToTarget the desired distance between the robot and the
	 *        target position
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for turning the robot to the specified target
	 *         position
	 */
	public static DriveCommand moveToward(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimaitonSubsystem, Translation2d targetPosition, double distanceToTarget,
			double distanceTolerance, double angleTolerance) {
		return moveToward(
				driveSubsystem, poseEstimaitonSubsystem, () -> targetPosition, distanceToTarget, distanceTolerance,
				angleTolerance);
	}

	/**
	 * Constructs a {@code DriveCommand} for moving the robot toward the specified
	 * target position while ensuring that the robot is away from the target by the
	 * specified distance.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimaitonSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param targetPositionSupplier a {@code Supplier<Pose2d>} that provides the
	 *        target position.
	 *        This is used at the commencement of the {@code DriveCommand} (i.e.,
	 *        when the scheduler begins to periodically execute the
	 *        {@code DriveCommand})
	 * @param distanceToTarget the desired distance between the robot and the
	 *        target position
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for turning the robot to the specified target
	 *         position
	 */
	public static DriveCommand moveToward(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimaitonSubsystem, Supplier<Translation2d> targetPositionSupplier,
			double distanceToTarget, double distanceTolerance, double angleTolerance) {
		Supplier<Pose2d> poseSupplier = () -> poseEstimaitonSubsystem.getEstimatedPose();
		return new DriveCommand(driveSubsystem, poseSupplier,
				() -> {
					var transform = poseEstimaitonSubsystem
							.transformationToward(targetPositionSupplier.get(), distanceToTarget);
					return poseSupplier.get().plus(transform);
				},
				distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code DriveCommand} for turning the robot to the specified
	 * target.
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
	public static DriveCommand turnToClosestTag(DriveSubsystem driveSubsystem,
			PoseEstimationSubsystem poseEstimationSubsystem, double angleOfCoverageInDegrees,
			double distanceThresholdInMeters, double distanceTolerance, double angleTolerance) {
		return turnTo(
				driveSubsystem, poseEstimationSubsystem,
				() -> poseEstimationSubsystem.closestTagPose(angleOfCoverageInDegrees, distanceThresholdInMeters),
				distanceThresholdInMeters, angleTolerance);
	}

	/**
	 * Constructs a {@code DriveCommand} for moving the robot to the specified
	 * target.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseEstimationSubsystem the {@code PoseEstimationSubsystem} to use
	 * @param angleOfCoverageInDegrees the angular coverage (in degrees) within
	 *        which {@code AprilTag}s are considered (maximum: 180)
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @param robotToTarget the {@code Pose2d} of the target relative to the
	 *        {@code Pose2d} of the robot for the {@code DriveCommand} to finish
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for moving the robot to the specified
	 *         target
	 */
	public static DriveCommand moveToClosestTag(DriveSubsystem driveSubsystem,
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