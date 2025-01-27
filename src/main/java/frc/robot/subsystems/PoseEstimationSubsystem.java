package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotConstants.*;

import java.util.Map;
import java.util.Map.Entry;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimationSubsystem extends SubsystemBase {

	/**
	 * The {@code PhotonCamera}s used by this {@code PoseEstimationSubsystem}.
	 */
	private final PhotonCamera[] m_cameras;

	/**
	 * The {@code PhotonCamera} used by this {@code PoseEstimationSubsystem}.
	 */
	private final DriveSubsystem m_driveSubsystem;

	/**
	 * The standard deviations of model states (smaller values will cause the Kalman
	 * filter to more trust the estimates).
	 */
	private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

	/**
	 * The standard deviations of the vision measurements (smaller values will cause
	 * the Kalman filter to more trust the measurements).
	 */
	private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

	/**
	 * The {@code PhotonPoseEstimator} used by this {@code PoseEstimationSubsystem}
	 * to improve the accuracy when multiple tags are detected.
	 */
	private final PhotonPoseEstimator m_photonPoseEstimator;

	/**
	 * The {@code SwerveDrivePoseEstimator} used by this
	 * {@code PoseEstimationSubsystem} to improve accuracy by injecting odometry
	 * data while addressing delays via Kalman filtering.
	 */
	private final SwerveDrivePoseEstimator m_poseEstimator;

	/**
	 * The {@code StructPublisher} for reporting the detected {@code Pose2d} of the
	 * robot.
	 */
	private final StructPublisher<Pose2d> m_detectedPosePublisher;

	/**
	 * The {@code StructPublisher} for reporting the estimated {@code Pose2d} of the
	 * robot.
	 */
	private final StructPublisher<Pose2d> m_estimatedPosePublisher;

	/**
	 * Constructs a {@code PoseEstimationSubsystem}.
	 * 
	 * @param driveSubsystem {@code DriveSubsystem} to be used by the
	 *        {@code PoseEstimationSubsystem}
	 * @param camera the {@code PhotonCamera}s to be used by the
	 *        {@code PoseEstimationSubsystem} (found in the PhotonVision UI)
	 */
	public PoseEstimationSubsystem(DriveSubsystem driveSubsystem, PhotonCamera... cameras) {
		m_cameras = cameras;
		m_driveSubsystem = driveSubsystem;
		m_photonPoseEstimator = new PhotonPoseEstimator(kFieldLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCamera1);
		m_poseEstimator = new SwerveDrivePoseEstimator(
				driveSubsystem.kinematics(),
				driveSubsystem.getHeading(),
				driveSubsystem.getModulePositions(),
				new Pose2d(),
				stateStdDevs,
				visionMeasurementStdDevs);
		m_detectedPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PhotonPoseEstimator", Pose2d.struct)
				.publish();
		m_estimatedPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PoseEstimationSubsystem", Pose2d.struct)
				.publish();
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}.
	 */
	@Override
	public void periodic() {
		for (var c : m_cameras) // for every camera c
			for (var r : c.getAllUnreadResults()) { // for every result r
				Optional<EstimatedRobotPose> e = m_photonPoseEstimator.update(r); // assign estimated pose to e
				if (e.isPresent()) { // if successful
					EstimatedRobotPose v = e.get(); // get successfully estimated pose
					m_poseEstimator.addVisionMeasurement(v.estimatedPose.toPose2d(), v.timestampSeconds);
					m_detectedPosePublisher.set(v.estimatedPose.toPose2d());
				}
			}
		m_poseEstimator.update(m_driveSubsystem.getHeading(), m_driveSubsystem.getModulePositions());
		m_estimatedPosePublisher.set(m_poseEstimator.getEstimatedPosition());
	}

	/**
	 * Returns the most recent estimated {@code Pose2d} of the robot.
	 * 
	 * @return the most recent estimated {@code Pose2d} of the robot
	 */
	public Pose2d getEstimatedPose() {
		return m_poseEstimator.getEstimatedPosition();
	}

	/**
	 * Finds the {@code Pose2d} of the {@code AprilTag} that is closest to the robot
	 * ({@code null} if no such {@code AprilTag}).
	 * 
	 * @param pose a {@code Pose2d}
	 * @param angleOfCoverageInDegrees the angular coverage (in degrees) within
	 *        which {@code AprilTag}s are considered (maximum: 180)
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the {@code Pose2d} of the {@code AprilTag} that is closest to the
	 *         robot ({@code null} if no such {@code AprilTag})
	 */
	public Pose2d closestTagPose(double angleOfCoverageInDegrees, double distanceThresholdInMeters) {
		var i = closestTagID(getEstimatedPose(), angleOfCoverageInDegrees, distanceThresholdInMeters);
		return i == null ? null : kFieldLayout.getTagPose(i).get().toPose2d();
	}

	/**
	 * Determines the ID of the {@code AprilTag} that is closest to the robot
	 * ({@code null} if no such {@code AprilTag}).
	 * 
	 * @param pose a {@code Pose2d}
	 * @param angleOfCoverageInDegrees the angular coverage (in degrees) within
	 *        which {@code AprilTag}s are considered (maximum: 180)
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the ID of the {@code AprilTag} that is closest to the robot
	 *         ({@code null} if no such {@code AprilTag})
	 */
	public Integer closestTagID(double angleOfCoverageInDegrees, double distanceThresholdInMeters) {
		return closestTagID(getEstimatedPose(), angleOfCoverageInDegrees, distanceThresholdInMeters);
	}

	/**
	 * Calculates the translational displacement from the robot to the target
	 * (negative displacement: backward).
	 * 
	 * @param targetPose the {@code Pose2d} of the target
	 * @return the translational displacement from the robot to the target
	 *         (negative displacement: backward)
	 */
	public double translationalDisplacement(Pose2d targetPose) {
		return translationalDisplacement(getEstimatedPose(), targetPose);
	}

	/**
	 * Calculates the angular displacement from the robot to the target.
	 * 
	 * @param targetPose the last {@code Pose2d}
	 * @return the angular displacement from the robot to the target
	 */
	public Rotation2d angularDisplacement(Pose2d targetPose) {
		return angularDisplacement(getEstimatedPose(), targetPose);
	}

	/**
	 * Returns the transformation needed for the robot to face toward the specified
	 * target position and be the specified distance away fron the target
	 * position.
	 * 
	 * @param targetPosition the target position whose x and y-coordinate values
	 *        are in meters
	 * @param distanceToTarget the desired distance in meters to the target
	 * @return the transformation needed for the robot to face toward the specified
	 *         target position and be the specified distance away fron the
	 *         target position; {@code null} if it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public Transform2d transformationToward(Translation2d targetPosition,
			double distanceToTarget) {
		return transformationToward(targetPosition, getEstimatedPose(), distanceToTarget);
	}

	/**
	 * Determines the ID of the {@code AprilTag} that is closest to the specified
	 * {@code Pose2d} ({@code null} if no such {@code AprilTag}).
	 * 
	 * @param pose a {@code Pose2d}
	 * @param angleOfCoverageInDegrees the angular coverage (in degrees) within
	 *        which {@code AprilTag}s are considered (maximum: 180)
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the ID of the {@code AprilTag} that is closest to the specified
	 *         {@code Pose2d} ({@code null} if no such {@code AprilTag})
	 */
	public static Integer closestTagID(Pose2d pose, double angleOfCoverageInDegrees, double distanceThresholdInMeters) {
		var s = kFieldLayout.getTags().stream()
				// consider only the tags facing toward the robot
				.filter(
						t -> Math.abs(
								t.pose.getTranslation().toTranslation2d().minus(pose.getTranslation()).getAngle()
										.minus(t.pose.toPose2d().getRotation()).getDegrees()) > 90)
				.filter( // consider only the tags within the angle of coverage
						t -> Math.abs(
								angularDisplacement(pose, t.pose.toPose2d()).getDegrees()) < angleOfCoverageInDegrees)
				.map(t -> Map.entry(t.ID, Math.abs(translationalDisplacement(pose, t.pose.toPose2d())))) // distance
				.filter(t -> t.getValue() < distanceThresholdInMeters); // only tags sufficently close
		Optional<Entry<Integer, Double>> closest = s.reduce((e1, e2) -> e1.getValue() < e2.getValue() ? e1 : e2);
		if (closest.isPresent()) {
			return closest.get().getKey();
		} else
			return null;
	}

	/**
	 * Calculates the translational displacement from the initial {@code Pose2d} to
	 * the last {@code Pose2d}.
	 * 
	 * @param initial the initial {@code Pose2d}
	 * @param last the last {@code Pose2d}
	 * @return the translational displacement from the initial {@code Pose2d} to
	 *         the last {@code Pose2d}
	 */
	public static double translationalDisplacement(Pose2d initial, Pose2d last) {
		var t = last.getTranslation().minus(initial.getTranslation());
		return Math.abs(t.getAngle().minus(initial.getRotation()).getDegrees()) > 90
				? -t.getNorm()
				: t.getNorm();
	}

	/**
	 * Calculates the angular displacement from the initial {@code Pose2d} to
	 * the last {@code Pose2d}.
	 * 
	 * @param initial the initial {@code Pose2d}
	 * @param last the last {@code Pose2d}
	 * @return the angular displacement from the initial {@code Pose2d} to
	 *         the last {@code Pose2d}
	 */
	public static Rotation2d angularDisplacement(Pose2d initial, Pose2d last) {
		var t = last.getTranslation().minus(initial.getTranslation());
		return t.getAngle().minus(initial.getRotation());
	}

	/**
	 * Returns the transformation needed for the robot to face toward the specified
	 * target position and be the specified distance away fron the target
	 * position.
	 * 
	 * @param targetPosition the target position whose x and y-coordinate values
	 *        are in meters
	 * @param currentPose the current {@code Pose2d} of the robot
	 * @param distanceToTarget the desired distance in meters to the target
	 * @return the transformation needed for the robot to face toward the specified
	 *         target position and be the specified distance away fron the
	 *         target position; {@code null} if it has not been
	 *         possible to reliably estimate the pose of the robot
	 */
	public static Transform2d transformationToward(Translation2d targetPosition, Pose2d currentPose,
			double distanceToTarget) {
		Translation2d diff = targetPosition.minus(currentPose.getTranslation());
		if (diff.getNorm() == 0)
			return null;
		var targetPose = new Pose2d(
				currentPose.getTranslation().plus(diff.times(1 - distanceToTarget / diff.getNorm())),
				diff.getAngle());
		return targetPose.minus(currentPose);
	}

	/**
	 * Constructs a {@code Pose2d}.
	 * 
	 * @param x the x component of the {@code Pose2d}
	 * @param y the y component of the {@code Pose2d}
	 * @param yawInDegrees the rotational component of the {@code Pose2d} in degrees
	 * @return the constructed {@code Pose2d}
	 */
	public static Pose2d pose(double x, double y, double yawInDegrees) {
		return new Pose2d(x, y, Rotation2d.fromDegrees(yawInDegrees));
	}

	/**
	 * Constructs a {@code Rotation2d}.
	 * 
	 * @param angleInDegrees the angle of the {@code Rotation2d} in
	 *        degrees
	 * @return the constructed {@code Rotation2d}
	 */
	public static Rotation2d rotation(double angleInDegrees) {
		return Rotation2d.fromDegrees(angleInDegrees);
	}

}