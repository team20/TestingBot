package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This {@code ChainableDriveCommand} aims to maneuver the robot to a certain
 * {@code Pose2d}.
 * It utilizes two {@code ProfiledPIDController}s to precisely control the
 * robot in the x, y, and yaw dimensions. A {@code ChainableDriveCommand} can be
 * used after a {@code ChainableDriveCommand} for
 * optimizations through re-use of the {@code ProfiledPIDController}s of the
 * previous {@code ChainableDriveCommand}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class ChainableDriveCommand2Controllers extends DriveCommand2Controllers {

	/**
	 * The {@code ChainableDriveCommand} right before this
	 * {@code ChainableDriveCommand}.
	 */
	private ChainableDriveCommand2Controllers m_previous;

	/**
	 * Constructs a new {@code ChainableDriveCommand} whose purpose is to move the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param targetPose the {@code Pose2d} to which the robot needs to move
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 */
	public ChainableDriveCommand2Controllers(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
			double angleTolerance) {
		this(driveSubsystem, () -> driveSubsystem.getPose(), () -> targetPose, distanceTolerance, angleTolerance);
		m_previous = null;
	}

	/**
	 * Constructs a new {@code ChainableDriveCommand} whose purpose is to move the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code ChainableDriveCommand} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code ChainableDriveCommand})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 */
	public ChainableDriveCommand2Controllers(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier,
			Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance,
			double angleToleranceInDegrees) {
		super(driveSubsystem, poseSupplier, targetPoseSupplier, false, distanceTolerance, angleToleranceInDegrees);
		m_previous = null;
	}

	/**
	 * Constructs a new {@code ChainableDriveCommand} whose purpose is to move the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param targetPose the {@code Pose2d} to which the robot needs to move
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @param previous the {@code DriveCommand} right before the new
	 *        {@code ChainableDriveCommand}
	 */
	public ChainableDriveCommand2Controllers(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
			double angleToleranceInDegrees, ChainableDriveCommand2Controllers previous) {
		this(driveSubsystem, () -> driveSubsystem.getPose(), () -> targetPose, distanceTolerance,
				angleToleranceInDegrees,
				previous);
		m_previous = previous;
	}

	/**
	 * Constructs a new {@code ChainableDriveCommand} whose purpose is to move the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code ChainableDriveCommand} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code ChainableDriveCommand})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @param previous the {@code DriveCommand} right before the new
	 *        {@code ChainableDriveCommand}
	 */
	public ChainableDriveCommand2Controllers(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier,
			Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance,
			double angleToleranceInDegrees, ChainableDriveCommand2Controllers previous) {
		super(driveSubsystem, poseSupplier, targetPoseSupplier, false, distanceTolerance, angleToleranceInDegrees,
				previous.m_controllerXY, previous.m_controllerYaw);
		m_previous = previous;
	}

	/**
	 * Is invoked at the commencement of this {@code ChainableDriveCommand} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code ChainableDriveCommand}).
	 */
	@Override
	public void initialize() {
		Pose2d pose = m_poseSupplier.get();
		m_targetPose = pose;
		try {
			m_targetPose = m_targetPoseSupplier.get();
		} catch (Exception e) {
			e.printStackTrace();
		}
		m_controllerXY.setTolerance(m_distanceTolerance);
		m_controllerYaw.setTolerance(m_angleTolerance);
		if (m_previous == null) {
			m_controllerXY.reset(m_targetPose.minus(pose).getTranslation().getNorm());
			m_controllerYaw.reset(pose.getRotation().getRadians());
		}
	}

}