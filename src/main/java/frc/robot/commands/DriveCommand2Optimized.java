package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This {@code DriveCommand2Optimized} aims to maneuver the robot to a certain
 * {@code Pose2d}.
 * It utilizes two {@code ProfiledPIDController}s to precisely control the
 * robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DriveCommand2Optimized extends DriveCommand2 {

	/**
	 * The distance error in meters which is tolerable.
	 */
	private double m_distanceTolerance;

	/**
	 * The angle error in degrees which is tolerable.
	 */
	private double m_angleTolerance;

	/**
	 * Constructs a new {@code DriveCommand2Optimized} whose purpose is to move the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param targetPose the {@code Pose2d} to which the robot needs to move
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param previous the {@code DriveCommand2} right before the new
	 *        {@code DriveCommand2Optimized}
	 */
	public DriveCommand2Optimized(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
			double angleTolerance, DriveCommand2 previous) {
		this(driveSubsystem, () -> driveSubsystem.getPose(), () -> targetPose, distanceTolerance, angleTolerance,
				previous);
	}

	/**
	 * Constructs a new {@code DriveCommand2Optimized} whose purpose is to move the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code DriveCommand2Optimized} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code DriveCommand2Optimized})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param previous the {@code DriveCommand2} right before the new
	 *        {@code DriveCommand2Optimized}
	 */
	public DriveCommand2Optimized(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier,
			Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance,
			double angleTolerance, DriveCommand2 previous) {
		super(driveSubsystem, poseSupplier, targetPoseSupplier, distanceTolerance, angleTolerance,
				previous.m_controllerXY, previous.m_controllerYaw);
		m_distanceTolerance = distanceTolerance;
		m_angleTolerance = angleTolerance;
	}

	/**
	 * Is invoked at the commencement of this {@code DriveCommand2Optimized} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code DriveCommand2Optimized}).
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
		m_controllerXY.setGoal(0);
		m_controllerYaw.setGoal(m_targetPose.getRotation().getDegrees());
	}

}