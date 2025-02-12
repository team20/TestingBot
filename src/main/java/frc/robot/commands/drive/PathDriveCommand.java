package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.*;

import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This {@code PathDriveCommand} aims to maneuver the robot to certain
 * {@code Pose2d}s. It utilizes two {@code PIDController}s to precisely
 * control the robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class PathDriveCommand extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code PathDriveCommand}.
	 */
	protected DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code PIDController} for controlling the robot in the x and y
	 * dimensions in meters (input: error in meters, output: velocity in meters per
	 * second).
	 */
	protected PIDController m_controllerXY;

	/**
	 * The {@code PIDController} for controlling the robot in the yaw dimension in
	 * radians (input: error in radians, output: velocity in radians
	 * per second).
	 */
	protected PIDController m_controllerYaw;

	/**
	 * The distance error in meters which is tolerable.
	 */
	protected double m_distanceTolerance;

	/**
	 * The angle error in radians which is tolerable.
	 */
	protected double m_angleTolerance;

	/**
	 * The {@code Supplier<Pose2d>}s that provide the {@code Pose2d}s to which the
	 * robot should move.
	 */
	protected List<Supplier<Pose2d>> m_targetPoseSuppliers;

	/**
	 * The index for indicating the current {@code Pose2d} to which the robot
	 * should move.
	 */
	protected int m_targetPoseIndex = 0;

	/**
	 * The current target {@code Pose2d} to which the robot should move.
	 */
	protected Pose2d m_targetPose;

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
				Arrays.stream(targetPoses).map(p -> (Supplier<Pose2d>) (() -> p)).toList());
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
			@SuppressWarnings("unchecked") Supplier<Pose2d>... targetPoseSuppliers) {
		this(driveSubsystem, distanceTolerance, angleToleranceInDegrees, intermediateToleranceRatio,
				Arrays.stream(targetPoseSuppliers).toList());
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
			List<Supplier<Pose2d>> targetPoseSuppliers) {
		m_driveSubsystem = driveSubsystem;
		m_targetPoseSuppliers = targetPoseSuppliers;
		m_distanceTolerance = distanceTolerance;
		m_angleTolerance = Math.toRadians(angleToleranceInDegrees);
		m_intermediateToleranceRatio = intermediateToleranceRatio;
		m_controllerXY = new PIDController(kDriveP, kDriveI, kDriveD);
		m_controllerYaw = new PIDController(kTurnP, kTurnI, kTurnD);
		m_controllerYaw.enableContinuousInput(0, 2 * Math.PI);
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code PathDriveCommand} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code PathDriveCommand}).
	 */
	@Override
	public void initialize() {
		setTargetPose(0);
	}

	/**
	 * Sets the current target {@code Pose2d}
	 * 
	 * @param targetPoseIndex the index of the target {@code Pose2d}
	 */
	void setTargetPose(int targetPoseIndex) {
		m_targetPoseIndex = targetPoseIndex;
		Pose2d pose = m_driveSubsystem.getPose();
		m_targetPose = pose;
		try {
			m_targetPose = m_targetPoseSuppliers.get(m_targetPoseIndex).get();
		} catch (Exception e) {
			e.printStackTrace();
			m_targetPose = pose;
		}
		if (m_targetPoseIndex < m_targetPoseSuppliers.size() - 1) {
			m_controllerXY.setTolerance(m_intermediateToleranceRatio * m_distanceTolerance);
			m_controllerYaw.setTolerance(m_intermediateToleranceRatio * m_angleTolerance);
		} else {
			m_controllerXY.setTolerance(m_distanceTolerance);
			m_controllerYaw.setTolerance(m_angleTolerance);
		}
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code PathDriveCommand} is either ended or interrupted.
	 */
	@Override
	public void execute() {
		m_driveSubsystem.drive(chassisSpeeds(), true);
	}

	/**
	 * Calculates the {@code ChassisSpeeds} to move the robot toward the current
	 * target.
	 * 
	 * @return the calculated {@code ChassisSpeeds} to move the robot toward the
	 *         current target
	 */
	public ChassisSpeeds chassisSpeeds() {
		var currentPose = m_driveSubsystem.getPose();
		Translation2d current2target = m_targetPose.getTranslation()
				.minus(currentPose.getTranslation());
		double velocityX = 0, velocityY = 0;
		try {
			double distance = current2target.getNorm();
			double speed = Math.abs(m_controllerXY.calculate(distance, 0));
			speed = applyThreshold(speed, kDriveMinSpeed);
			var angle = current2target.getAngle();
			velocityX = speed * angle.getCos();
			velocityY = speed * angle.getSin();
		} catch (Exception e) {
		}
		double angularVelocityRadiansPerSecond = m_controllerYaw
				.calculate(currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians());
		angularVelocityRadiansPerSecond = applyThreshold(angularVelocityRadiansPerSecond, kTurnMinAngularSpeed);
		return new ChassisSpeeds(velocityX, velocityY, angularVelocityRadiansPerSecond);
	}

	/**
	 * Is invoked once this {@code PathDriveCommand} is either ended or
	 * interrupted.
	 * 
	 * @param interrupted indicates if this {@code PathDriveCommand} was
	 *        interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.drive(0, 0, 0, true);
	}

	/**
	 * Determines whether or not this {@code PathDriveCommand} needs to end.
	 * 
	 * @return {@code true} if this {@code PathDriveCommand} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		if (m_controllerXY.atSetpoint() && m_controllerYaw.atSetpoint()) {
			if (m_targetPoseIndex == m_targetPoseSuppliers.size() - 1)
				return true;
			setTargetPose(m_targetPoseIndex + 1);
		}
		return false;
	}

	/**
	 * Applies the specified threshold to the specified value.
	 * 
	 * @param value the value to be thresholded
	 * @param threshold the threshold limit
	 * @return the original value if the absolute value of that value is greater or
	 *         equal to the threshold; the threshold with the original value's sign
	 *         otherwise
	 */
	public static double applyThreshold(double value, double threshold) {
		return Math.abs(value) < threshold ? Math.signum(value) * threshold : value;
	}

}