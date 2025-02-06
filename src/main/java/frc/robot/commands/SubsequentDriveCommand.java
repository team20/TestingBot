package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This {@code SubsequentDriveCommand} aims to maneuver the robot to a certain
 * {@code Pose2d}.
 * It utilizes two {@code ProfiledPIDController}s to precisely control the
 * robot in the x, y, and yaw dimensions. A {@code SubsequentDriveCommand} is a
 * {@code DriveCommand} and is always used after a {@code DriveCommand} for
 * optimizations through re-use of the {@code ProfiledPIDController}s of the
 * previous {@code DriveCommand}.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class SubsequentDriveCommand extends DriveCommand {

	/**
	 * Constructs a new {@code SubsequentDriveCommand} whose purpose is to move the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param targetPose the {@code Pose2d} to which the robot needs to move
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param previous the {@code DriveCommand} right before the new
	 *        {@code SubsequentDriveCommand}
	 */
	public SubsequentDriveCommand(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
			double angleTolerance, DriveCommand previous) {
		this(driveSubsystem, () -> driveSubsystem.getPose(), () -> targetPose, distanceTolerance, angleTolerance,
				previous);
	}

	/**
	 * Constructs a new {@code SubsequentDriveCommand} whose purpose is to move the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code SubsequentDriveCommand} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code SubsequentDriveCommand})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param previous the {@code DriveCommand} right before the new
	 *        {@code SubsequentDriveCommand}
	 */
	public SubsequentDriveCommand(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier,
			Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance,
			double angleTolerance, DriveCommand previous) {
		super(driveSubsystem, poseSupplier, targetPoseSupplier, distanceTolerance, angleTolerance,
				previous.m_controllerXY, previous.m_controllerYaw);
	}

	/**
	 * Is invoked at the commencement of this {@code SubsequentDriveCommand} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code SubsequentDriveCommand}).
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
	}

}