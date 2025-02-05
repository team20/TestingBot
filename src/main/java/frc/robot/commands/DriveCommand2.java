package frc.robot.commands;

import static frc.robot.Constants.DriveConstants.*;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This {@code DriveCommand2} aims to maneuver the robot to a certain
 * {@code Pose2d}.
 * It utilizes two {@code ProfiledPIDController}s to precisely control the
 * robot in the x, y, and yaw dimensions.
 * 
 * @author Jeong-Hyon Hwang (jhhbrown@gmail.com)
 * @author Andrew Hwang (u.andrew.h@gmail.com)
 */
public class DriveCommand2 extends Command {

	/**
	 * The {@code DriveSubsystem} used by this {@code DriveCommand2}.
	 */
	protected DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code Supplier} providing the current {@code Pose2d} of the robot.
	 */
	protected Supplier<Pose2d> m_poseSupplier;

	/**
	 * The {@code Supplier<Pose2d>} that provides the {@code Pose2d} to which the
	 * robot should move.
	 * This is used at the commencement of this {@code DriveCommand2} (i.e.,
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
	 * dimension in degrees (input: error in degrees, output: velocity in radians
	 * per second).
	 */
	protected ProfiledPIDController m_controllerYaw;

	/**
	 * The {@code Pose2d} to which the robot should move.
	 */
	protected Pose2d m_targetPose;

	/**
	 * Constructs a new {@code DriveCommand2} whose purpose is to move the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param targetPose the {@code Pose2d} to which the robot needs to move
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 */
	public DriveCommand2(DriveSubsystem driveSubsystem, Pose2d targetPose, double distanceTolerance,
			double angleTolerance) {
		this(driveSubsystem, () -> driveSubsystem.getPose(), () -> targetPose, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a {@code DriveCommand2} for moving the robot forward/backward by
	 * the specified displacement.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param translationalDisplacement the displacement by which the robot to move
	 *        (positive: forward, negative: backward) in meters
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @return a {@code DriveCommand2} for moving the robot to the specified
	 *         target
	 */
	public static DriveCommand2 moveForward(DriveSubsystem driveSubsystem, double translationalDisplacement,
			double distanceTolerance, double angleTolerance) {
		return new DriveCommand2(driveSubsystem, () -> driveSubsystem.getPose(), () -> {
			var transform = new Transform2d(translationalDisplacement, 0, Rotation2d.kZero);
			return driveSubsystem.getPose().plus(transform);
		}, distanceTolerance, angleTolerance);
	}

	/**
	 * Constructs a new {@code DriveCommand2} whose purpose is to move the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code DriveCommand2} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code DriveCommand2})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 */
	public DriveCommand2(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier,
			Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance,
			double angleTolerance) {
		this(driveSubsystem, poseSupplier, targetPoseSupplier, distanceTolerance, angleTolerance,
				new ProfiledPIDController(kDriveP, kDriveI, kDriveD,
						new TrapezoidProfile.Constraints(kDriveMaxSpeed, kDriveMaxAcceleration)),
				new ProfiledPIDController(kTurnP, kTurnI, kTurnD,
						new TrapezoidProfile.Constraints(Math.toDegrees(kTurnMaxAngularSpeed),
								Math.toDegrees(kTurnMaxAcceleration))));
	}

	/**
	 * Constructs a new {@code DriveCommand2} whose purpose is to move the
	 * robot to a certain {@code Pose2d}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @param poseSupplier the {@code Supplier} providing the current {@code Pose2d}
	 *        of the robot
	 * @param targetPoseSupplier a {@code Supplier<Pose2d>} that provides the
	 *        {@code Pose2d} to which the robot should move.
	 *        This is used at the commencement of this
	 *        {@code DriveCommand2} (i.e., when the scheduler
	 *        begins to periodically execute this
	 *        {@code DriveCommand2})
	 * @param distanceTolerance the distance error in meters which is tolerable
	 * @param angleTolerance the angle error in degrees which is tolerable
	 * @param controllerXY the {@code ProfiledPIDController} for controlling the
	 *        robot in the x and y dimensions in meters
	 * @param controllerYaw the {@code ProfiledPIDController} for controlling the
	 *        robot in the yaw dimension in angles
	 * 
	 */
	public DriveCommand2(DriveSubsystem driveSubsystem, Supplier<Pose2d> poseSupplier,
			Supplier<Pose2d> targetPoseSupplier,
			double distanceTolerance,
			double angleTolerance, ProfiledPIDController controllerXY, ProfiledPIDController controllerYaw) {
		m_driveSubsystem = driveSubsystem;
		m_poseSupplier = poseSupplier;
		m_targetPoseSupplier = targetPoseSupplier;
		m_controllerXY = controllerXY;
		m_controllerYaw = controllerYaw;
		m_controllerXY.setTolerance(distanceTolerance);
		m_controllerYaw.setTolerance(angleTolerance);
		m_controllerYaw.enableContinuousInput(-180, 180);
		addRequirements(m_driveSubsystem);
	}

	/**
	 * Is invoked at the commencement of this {@code DriveCommand2} (i.e,
	 * when the scheduler begins to periodically execute this
	 * {@code DriveCommand2}).
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
		Translation2d t = m_targetPose.getTranslation().minus(pose.getTranslation());
		m_controllerXY.reset(t.getNorm());
		m_controllerYaw.reset(pose.getRotation().getDegrees());
		m_controllerXY.setGoal(0);
		m_controllerYaw.setGoal(m_targetPose.getRotation().getDegrees());
	}

	/**
	 * Is invoked periodically by the scheduler until this
	 * {@code DriveCommand2} is either ended or interrupted.
	 */
	@Override
	public void execute() {
		Pose2d pose = m_poseSupplier.get();
		Translation2d t = m_targetPose.getTranslation().minus(pose.getTranslation());
		double speed = m_controllerXY.calculate(t.getNorm());
		t = t.getNorm() > 0 ? t.times(-speed / t.getNorm()) : t; // translation toward target
		double speedX = t.getX();
		double speedY = t.getY();
		// NEGATION needed if the robot rotates clockwise given positive turnSpeed
		double speedYaw = m_controllerYaw.calculate(pose.getRotation().getDegrees());
		speedX = applyThreshold(speedX, kDriveMinSpeed);
		speedY = applyThreshold(speedY, kDriveMinSpeed);
		speedYaw = applyThreshold(speedYaw, kTurnMinAngularSpeed);
		m_driveSubsystem.drive(speedX, speedY, speedYaw, true);
	}

	/**
	 * Is invoked once this {@code DriveCommand2} is either ended or interrupted.
	 * 
	 * @param interrupted indicates if this {@code DriveCommand2} was interrupted
	 */
	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.drive(0, 0, 0, true);
	}

	/**
	 * Determines whether or not this {@code DriveCommand2} needs to end.
	 * 
	 * @return {@code true} if this {@code DriveCommand2} needs to end;
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

	/**
	 * Returns a {@code Command} for testing the {@code DriveCommand2}
	 * implementation.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to use
	 * @return a {@code Command} for testing the {@code DriveCommand2}
	 *         implementation
	 */
	public static Command testCommand(DriveSubsystem driveSubsystem) {
		return new DriveCommand(driveSubsystem, new Pose2d(.5, .5, Rotation2d.fromDegrees(30)), .1, 3)
				.andThen(new DriveCommand(driveSubsystem, Pose2d.kZero, .1, 3));
	}
}