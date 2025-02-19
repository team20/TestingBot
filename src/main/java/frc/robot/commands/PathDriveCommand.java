package frc.robot.commands;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This {@code PathDriveCommand} aims to maneuver the robot to certain
 * {@code Pose2d}s. It utilizes two {@code PIDController}s to precisely
 * control the robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class PathDriveCommand extends DriveCommand {

	/**
	 * The ratio to apply to the distance and angle tolerances for intermeidate
	 * target {@code Pose2d}s
	 */
	protected double m_intermediateToleranceRatio;

	/**
	 * Constructs a new {@code PathDriveCommand} whose purpose is to move
	 * the robot to certain {@code Pose2d}s.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @param intermediateToleranceRatio the ratio to apply to the distance and
	 *        angle tolerances for intermeidate target {@code Pose2d}s
	 * @param targetPoses {@code Pose2d}s to which the robot should move
	 */
	public PathDriveCommand(DriveSubsystem driveSubsystem,
			double distanceTolerance,
			double angleToleranceInDegrees, double intermediateToleranceRatio,
			Pose2d... targetPoses) {
		this(driveSubsystem, distanceTolerance, angleToleranceInDegrees, intermediateToleranceRatio,
				Arrays.stream(targetPoses).toList());
	}

	/**
	 * Constructs a new {@code PathDriveCommand} whose purpose is to move
	 * the robot to certain {@code Pose2d}s.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @param intermediateToleranceRatio the ratio to apply to the distance and
	 *        angle tolerances for intermeidate target {@code Pose2d}s
	 * @param targetPoseSuppliers {@code Supplier<Pose2d>}s that provide the
	 *        {@code Pose2d}s to which the robot should move
	 */
	public PathDriveCommand(DriveSubsystem driveSubsystem,
			double distanceTolerance,
			double angleToleranceInDegrees, double intermediateToleranceRatio,
			Supplier<List<Pose2d>> targetPoseSuppliers) {
		super(driveSubsystem, driveSubsystem::getPose, targetPoseSuppliers, angleToleranceInDegrees, distanceTolerance);
		m_intermediateToleranceRatio = intermediateToleranceRatio;
	}

	/**
	 * Is invoked at the commencement of this {@code PathDriveCommand} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code PathDriveCommand}).
	 */
	@Override
	public void initialize() {
		targetPoseSupplier().reset();
		moveToNextTargetPose();
		Pose2d pose = m_driveSubsystem.getPose();
		m_controllerXY.reset(m_targetPose.minus(pose).getTranslation().getNorm());
		m_controllerYaw.reset(pose.getRotation().getRadians());
	}

	/**
	 * Moves to the next target {@code Pose2d}.
	 */
	protected void moveToNextTargetPose() {
		Pose2d pose = m_driveSubsystem.getPose();
		m_targetPose = pose;
		try {
			m_targetPose = m_targetPoseSupplier.get();
		} catch (Exception e) {
			e.printStackTrace();
			m_targetPose = pose;
		}
		if (targetPoseSupplier().hasNext()) {
			m_controllerXY.setTolerance(m_intermediateToleranceRatio * m_distanceTolerance);
			m_controllerYaw.setTolerance(m_intermediateToleranceRatio * m_angleTolerance);
		} else {
			m_controllerXY.setTolerance(m_distanceTolerance);
			m_controllerYaw.setTolerance(m_angleTolerance);
		}
	}

	/**
	 * Determines whether or not this {@code PathDriveCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code PathDriveCommand} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		if (alignedTo(m_targetPose)) {
			if (targetPoseSupplier().hasNext())
				moveToNextTargetPose();
			else
				return true;
		}
		return false;
	}

	/**
	 * Determines whether or not the robot is sufficiently aligned to the specified
	 * target {@code Pose2d}.
	 * 
	 * @param targetPose the target {@code Pose2d}
	 * @return {@code true} if the robot is sufficiently aligned to the specified
	 *         target {@code Pose2d}; {@code false} otherwise
	 */
	boolean alignedTo(Pose2d targetPose) {
		var diff = targetPose.minus(m_driveSubsystem.getPose());
		return diff.getTranslation().getNorm() < m_controllerXY.getPositionTolerance()
				&& Math.abs(diff.getRotation().getRadians()) < m_controllerYaw.getPositionTolerance();
	}
}