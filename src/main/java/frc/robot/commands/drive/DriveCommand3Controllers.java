package frc.robot.commands.drive;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This {@code DriveCommand3Controllers} aims to maneuver the robot to a certain
 * {@code Pose2d}.
 * It utilizes three {@code ProfiledPIDController}s to precisely control the
 * robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DriveCommand3Controllers extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code DriveCommand3Controllers}.
	 */
	private DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code Supplier} providing the current {@code Pose2d} of the robot.
	 */
	protected Supplier<Pose2d> m_poseSupplier;

	/**
	 * The {@code Supplier<Pose2d>} that provides the {@code Pose2d} to which the
	 * robot should move.
	 * This is used at the commencement of this {@code DriveCommand3Controllers}
	 * (i.e.,
	 * when the scheduler begins to periodically execute this {@code
	 * DriveCommand}).
	 */
	protected Supplier<Pose2d> m_targetPoseSupplier;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the x
	 * dimension in meters (input: error in meters, output: velocity in meters per
	 * second).
	 */
	protected ProfiledPIDController m_controllerX;

	/**
	 * The {@code ProfiledPIDController} for controlling the robot in the y
	 * dimension in meters (input: error in meters, output: velocity in meters per
	 * second).
	 */
	protected ProfiledPIDController m_controllerY;

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
	 * Constructs a new {@code DriveCommand3Controllers} whose purpose is to move
	 * the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param targetPose the {@code Pose2d} to which the robot needs to move
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 */
	public DriveCommand3Controllers(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
			double angleTolerance) {
		this(driveSubsystem, () -> driveSubsystem.getPose(), () -> targetPose, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code DriveCommand3Controllers} for moving the robot
	 * forward/backward by the specified displacement.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param translationalDisplacement the displacement by which the robot to move
	 *        (positive: forward, negative: backward) in meters
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code Commmand} for moving the robot to the specified
	 *         target
	 */
	public static DriveCommand3Controllers moveForward(DriveSubsystem driveSubsystem, double translationalDisplacement,
			double distanceTolerance, double angleTolerance) {
		return new DriveCommand3Controllers(driveSubsystem, () -> driveSubsystem.getPose(), () -> {
			var transform = new Transform2d(translationalDisplacement, 0, Rotation2d.kZero);
			return driveSubsystem.getPose().plus(transform);
		}, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand3Controllers} whose purpose is to move
	 * the robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code DriveCommand3Controllers} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code DriveCommand3Controllers})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 */
	public DriveCommand3Controllers(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier,
			Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance,
			double angleTolerance) {
		this(driveSubsystem, poseSupplier, targetPoseSupplier, distanceTolerance, angleTolerance,
				new ProfiledPIDController(kDriveP, kDriveI, kDriveD,
						new TrapezoidProfile.Constraints(kDriveMaxSpeed, kDriveMaxAcceleration)),
				new ProfiledPIDController(kDriveP, kDriveI, kDriveD,
						new TrapezoidProfile.Constraints(kDriveMaxSpeed, kDriveMaxAcceleration)),
				new ProfiledPIDController(kTurnP, kTurnI, kTurnD,
						new TrapezoidProfile.Constraints(kTurnMaxAngularSpeed, kTurnMaxAcceleration)));
		m_controllerYaw.enableContinuousInput(0, 2 * Math.PI);
		m_controllerX.setGoal(0);
		m_controllerY.setGoal(0);
		m_controllerYaw.setGoal(0);
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Constructs a new {@code DriveCommand3Controllers} whose purpose is to move
	 * the robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code DriveCommand3Controllers} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code DriveCommand3Controllers})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 */
	public DriveCommand3Controllers(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier,
			Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance,
			double angleTolerance, ProfiledPIDController controllerX, ProfiledPIDController controllerY,
			ProfiledPIDController controllerYaw) {
		m_driveSubsystem = driveSubsystem;
		m_poseSupplier = poseSupplier;
		m_targetPoseSupplier = targetPoseSupplier;
		m_distanceTolerance = distanceTolerance;
		m_angleTolerance = Math.toRadians(angleTolerance);
		m_controllerX = controllerX;
		m_controllerY = controllerY;
		m_controllerYaw = controllerYaw;
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code DriveCommand3Controllers} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code DriveCommand3Controllers}).
	 */
	@Override
	public void initialize() {
		Pose2d pose = m_poseSupplier.get();
		m_targetPose = pose;
		try {
			m_targetPose = m_targetPoseSupplier.get();
		} catch (Exception e) {
		}
		m_controllerX.setTolerance(m_distanceTolerance);
		m_controllerY.setTolerance(m_distanceTolerance);
		m_controllerYaw.setTolerance(m_angleTolerance);
		var t = m_targetPose.getTranslation().minus(pose.getTranslation());
		m_controllerX.reset(t.getX());
		m_controllerY.reset(t.getY());
		m_controllerYaw.reset(m_targetPose.getRotation().minus(pose.getRotation()).getRadians());
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code DriveCommand3Controllers} is either ended or interrupted.
	 */
	@Override
	public void execute() {
		Pose2d pose = m_poseSupplier.get();
		var t = m_targetPose.getTranslation().minus(pose.getTranslation());
		double speedX = -m_controllerX.calculate(t.getX());
		double speedY = -m_controllerY.calculate(t.getY());
		double speedYaw = -m_controllerYaw.calculate(m_targetPose.getRotation().minus(pose.getRotation()).getRadians());
		speedX = applyThreshold(speedX, kDriveMinSpeed);
		speedY = applyThreshold(speedY, kDriveMinSpeed);
		speedYaw = applyThreshold(speedYaw, kTurnMinAngularSpeed);
		m_driveSubsystem.drive(speedX, speedY, speedYaw, true);
	}

	/**
	 * Is invoked once this {@code DriveCommand3Controllers} is either ended or
	 * interrupted.
	 * 
	 * @param interrupted indicates if this {@code DriveCommand3Controllers} was
	 *        interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.drive(0, 0, 0, true);
	}

	/**
	 * Determines whether or not this {@code DriveCommand3Controllers} needs to end.
	 * 
	 * @return {@code true} if this {@code DriveCommand3Controllers} needs to end;
	 *         {@code false} otherwise
	 */
	@Override
	public boolean isFinished() {
		return m_controllerX.atGoal() && m_controllerY.atGoal() && m_controllerYaw.atGoal();
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

	/**
	 * Returns a {@code Command} for testing the {@code DriveCommand3Controllers}
	 * implementation.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @return a {@code Command} for testing the {@code DriveCommand3Controllers}
	 *         implementation
	 */
	public static Command testCommand(DriveSubsystem driveSubsystem) {
		return new DriveCommand3Controllers(driveSubsystem, new Pose2d(.5, .5, Rotation2d.fromDegrees(30)), .1, 3)
				.andThen(new DriveCommand3Controllers(driveSubsystem, Pose2d.kZero, .1, 3));
	}
}