package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This {@code DriveCommand2Controllers} aims to maneuver the robot to a certain
 * {@code Pose2d}. It utilizes two {@code ProfiledPIDController}s to precisely
 * control the robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DriveCommand2Controllers extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code DriveCommand2Controllers}.
	 */
	protected DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code Supplier} providing the current {@code Pose2d} of the robot.
	 */
	protected Supplier<Pose2d> m_poseSupplier;

	/**
	 * The {@code Supplier<Pose2d>} that provides the {@code Pose2d} to which the
	 * robot should move.
	 * This is used at the commencement of this {@code DriveCommand2Controllers}
	 * (i.e.,
	 * when the scheduler begins to periodically execute this {@code
	 * DriveCommand}).
	 */
	protected Supplier<Pose2d> m_targetPoseSupplier;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the x and y
	 * dimensions in meters (input: error in meters, output: velocity in meters per
	 * second).
	 */
	protected ProfiledPIDController m_controllerXY;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the yaw
	 * dimension in radians (input: error in radians, output: velocity in radians
	 * per second).
	 */
	protected ProfiledPIDController m_controllerYaw;

	/**
	 * The {@code Pose2d} to which the robot should move.
	 */
	protected Pose2d m_targetPose;

	/**
	 * The distance error in meters which is tolerable.
	 */
	protected double m_distanceTolerance;

	/**
	 * The angle error in radians which is tolerable.
	 */
	protected double m_angleTolerance;

	/**
	 * Constructs a new {@code DriveCommand2Controllers} whose purpose is to move
	 * the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param targetPose the {@code Pose2d} to which the robot needs to move
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 */
	public DriveCommand2Controllers(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
			double angleToleranceInDegrees) {
		this(driveSubsystem, () -> driveSubsystem.getPose(), () -> targetPose, distanceTolerance,
				angleToleranceInDegrees);
	}

	/**
	 * Constructs a new {@code DriveCommand2Controllers} whose purpose is to move
	 * the robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code DriveCommand2Controllers} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code DriveCommand2Controllers})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 */
	public DriveCommand2Controllers(DriveSubsystem driveSubsystem, Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance, double angleToleranceInDegrees) {
		this(driveSubsystem, () -> driveSubsystem.getPose(), targetPoseSupplier, distanceTolerance,
				angleToleranceInDegrees);
	}

	/**
	 * Constructs a new {@code DriveCommand2Controllers} whose purpose is to move
	 * the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code DriveCommand2Controllers} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code DriveCommand2Controllers})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 */
	public DriveCommand2Controllers(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier,
			Supplier<Pose2d> targetPoseSupplier, double distanceTolerance, double angleToleranceInDegrees) {
		this(driveSubsystem, poseSupplier, targetPoseSupplier, distanceTolerance,
				angleToleranceInDegrees,
				new ProfiledPIDController(kDriveP, kDriveI, kDriveD,
						new TrapezoidProfile.Constraints(kDriveMaxSpeed, kDriveMaxAcceleration)),
				new ProfiledPIDController(kTurnP, kTurnI, kTurnD,
						new TrapezoidProfile.Constraints(kTurnMaxAngularSpeed,
								kTurnMaxAcceleration)));
		m_controllerYaw.enableContinuousInput(0, 2 * Math.PI);
	}

	/**
	 * Constructs a new {@code DriveCommand2Controllers} whose purpose is to move
	 * the robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code DriveCommand2Controllers} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code DriveCommand2Controllers})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleToleranceInDegrees the angle error in degrees which is tolerable
	 * @param controllerXY the {@code ProfiledPIDController} for controlling the
	 *        robot in the x and y dimensions in meters
	 * @param controllerYaw the {@code ProfiledPIDController} for controlling the
	 *        robot in the yaw dimension in angles
	 * 
	 */
	public DriveCommand2Controllers(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier,
			Supplier<Pose2d> targetPoseSupplier, double distanceTolerance,
			double angleToleranceInDegrees, ProfiledPIDController controllerXY, ProfiledPIDController controllerYaw) {
		m_driveSubsystem = driveSubsystem;
		m_poseSupplier = poseSupplier;
		m_targetPoseSupplier = targetPoseSupplier;
		m_distanceTolerance = distanceTolerance;
		m_angleTolerance = Math.toRadians(angleToleranceInDegrees);
		m_controllerXY = controllerXY;
		m_controllerYaw = controllerYaw;
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code DriveCommand2Controllers} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code DriveCommand2Controllers}).
	 */
	@Override
	public void initialize() {
		Pose2d pose = m_poseSupplier.get();
		m_targetPose = pose;
		try {
			m_targetPose = m_targetPoseSupplier.get();
		} catch (Exception e) {
			e.printStackTrace();
			m_targetPose = pose;
		}
		m_controllerXY.setTolerance(m_distanceTolerance);
		m_controllerYaw.setTolerance(m_angleTolerance);
		m_controllerXY.reset(m_targetPose.minus(pose).getTranslation().getNorm());
		m_controllerYaw.reset(pose.getRotation().getRadians());
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code DriveCommand2Controllers} is either ended or interrupted.
	 */
	@Override
	public void execute() {
		m_driveSubsystem.drive(chassisSpeeds(), true);
	}

	/**
	 * Calculates the {@code ChassisSpeeds} to move the robot toward the target.
	 * 
	 * @return the calculated {@code ChassisSpeeds} to move the robot toward the
	 *         target
	 */
	public ChassisSpeeds chassisSpeeds() {
		var currentPose = m_poseSupplier.get();
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
	 * Is invoked once this {@code DriveCommand2Controllers} is either ended or
	 * interrupted.
	 * 
	 * @param interrupted indicates if this {@code DriveCommand2Controllers} was
	 *        interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.drive(0, 0, 0, true);
	}

	/**
	 * Determines whether or not this {@code DriveCommand2Controllers} needs to end.
	 * 
	 * @return {@code true} if this {@code DriveCommand2Controllers} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		return m_controllerXY.atGoal() && m_controllerYaw.atGoal();
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