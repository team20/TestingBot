package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.VisionConstants.*;

import java.util.HashMap;
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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A {@code PoseEstimationSubsystem} continuously estimates the {@code Pose2d}
 * of the robot on the field.
 */
public class PoseEstimationSubsystem extends SubsystemBase {
	private final Map<PhotonCamera, PhotonPoseEstimator> m_cameras = new HashMap<PhotonCamera, PhotonPoseEstimator>();
	private final DriveSubsystem m_driveSubsystem;

	/**
	 * Standard deviations of the vision pose measurement (x position in meters, y
	 * position in meters, and heading in radians). Smaller values will cause
	 * the Kalman filter to trust the Vision data more.
	 */
	private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

	/**
	 * The {@code SwerveDrivePoseEstimator} used by this
	 * {@code PoseEstimationSubsystem} to improve accuracy by injecting odometry
	 * data while addressing delays via Kalman filtering.
	 */
	private final SwerveDrivePoseEstimator m_poseEstimator;

	private final StructPublisher<Pose2d> m_detectedPosePublisher;
	private final StructPublisher<Pose2d> m_estimatedPosePublisher;

	/**
	 * Constructs a {@code PoseEstimationSubsystem}.
	 * 
	 * @param driveSubsystem {@code DriveSubsystem} to be used by the
	 *        {@code PoseEstimationSubsystem}
	 */
	public PoseEstimationSubsystem(DriveSubsystem driveSubsystem) {
		m_driveSubsystem = driveSubsystem;
		m_poseEstimator = new SwerveDrivePoseEstimator(
				driveSubsystem.kinematics(),
				driveSubsystem.getHeading(),
				driveSubsystem.getModulePositions(),
				new Pose2d(),
				kStateStdDevs,
				visionMeasurementStdDevs);
		m_detectedPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PhotonPoseEstimator", Pose2d.struct)
				.publish();
		m_estimatedPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PoseEstimationSubsystem", Pose2d.struct)
				.publish();
	}

	/**
	 * Adds the specified {@code PhotonCamera} to this
	 * {@code PoseEstimationSubsystem}.
	 * 
	 * @param cameraSim a {@code PhotonCamera}
	 * @param robotToCamera the {@code Transform3d} expressing the pose of the
	 *        camera relative to the pose of the robot.
	 * @return this {@code PoseEstimationSubsystem}
	 */
	public PoseEstimationSubsystem addCamera(PhotonCamera camera, Transform3d robotToCamera) {
		m_cameras.put(
				camera, new PhotonPoseEstimator(kFieldLayout,
						PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera));
		return this;
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}.
	 */
	@Override
	public void periodic() {
		boolean firstCamera = true;
		for (var camEntry : m_cameras.entrySet()) {
			var camera = camEntry.getKey();
			var poseEstimator = camEntry.getValue();
			for (var result : camera.getAllUnreadResults()) { // for every result
				if (firstCamera || result.getTargets().size() > 1) { // TODO: use both camera results equally, don't
																		// prioritize cam1
					Optional<EstimatedRobotPose> pose = poseEstimator.update(result);
					if (pose.isPresent()) { // if successful
						EstimatedRobotPose estPose = pose.get(); // get successfully estimated pose
						m_poseEstimator
								.addVisionMeasurement(estPose.estimatedPose.toPose2d(), estPose.timestampSeconds);
						m_detectedPosePublisher.set(estPose.estimatedPose.toPose2d());
					}
				}
			}
			firstCamera = false;
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
	 * TODO: Do we need this too?
	 * 
	 * Returns the odometry-centric {@code Pose2d} of a specific target given the
	 * target's field-centric {@code Pose2d}.
	 * 
	 * @param fieldCentricPose a field-centric {@code Pose2d}
	 * @return the odometry-centric {@code Pose2d} that corresponds to the specified
	 *         field-centric {@code Pose2d}
	 */
	public Pose2d targetOdometryCentricPose(Pose2d fieldCentricPose) {
		return m_driveSubsystem.getPose().plus(fieldCentricPose.minus(getEstimatedPose()));
	}

	/**
	 * Finds the pose of the {@code AprilTag} that is closest to the robot, either
	 * in terms of distance or angle (based on user selection).
	 * 
	 * @param angleOfCoverage the angular coverage (in degrees) within
	 *        which AprilTags are considered (maximum: 180)
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which AprilTags are considered
	 * @param isClosestByDistance true for distance-based tag filtering.
	 *        false for angle-based tag filtering.
	 * @return the {@code Pose2d} of the {@code AprilTag} closest to the robot.
	 *         Returns null if there is no such AprilTag.
	 * @apiNote
	 */
	public Pose2d closestTagPose(Optional<Double> angleOfCoverage, double distanceThresholdInMeters,
			boolean isClosestByDistance) {
		var id = (isClosestByDistance)
				? distanceClosestTagID( // returns ID of tag closest in terms of distance
						getEstimatedPose(), distanceThresholdInMeters, angleOfCoverage)
				: angleClosestTagID( // returns ID of tag closest in terms of angular displacement
						getEstimatedPose(), distanceThresholdInMeters, angleOfCoverage);
		return id == null ? null : kFieldLayout.getTagPose(id).get().toPose2d();
	}

	/**
	 * Determines the ID of the {@code AprilTag} that is closest to the specified
	 * {@code Pose2d} ({@code null} if no such {@code AprilTag}).
	 * 
	 * @param pose a {@code Pose2d}
	 * @param angleOfCoverage the angular coverage (in degrees) within
	 *        which {@code AprilTag}s are considered (maximum: 180)
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the ID of the {@code AprilTag} that is closest to the specified
	 *         {@code Pose2d} ({@code null} if no such {@code AprilTag})
	 */
	public static Integer distanceClosestTagID(Pose2d pose, double distanceThresholdInMeters,
			Optional<Double> angleOfCoverage) {
		var s = kFieldLayout.getTags().stream();
		if (angleOfCoverage.isPresent()) {
			s.filter(
					tag -> (translationalDisplacement(pose, tag.pose.toPose2d()) < distanceThresholdInMeters
							&& translationalDisplacement(pose, tag.pose.toPose2d()) > 0));
		} else {
			s.filter(tag -> Math.abs(angularDisplacement(pose, tag.pose.toPose2d()).getDegrees()) < 90);
		}
		Optional<Entry<Integer, Double>> closest = s
				.map(tag -> Map.entry(tag.ID, Math.abs(translationalDisplacement(pose, tag.pose.toPose2d()))))
				.filter(tag -> tag.getValue() < distanceThresholdInMeters)
				.reduce((tag1, tag2) -> tag1.getValue() < tag2.getValue() ? tag1 : tag2);
		if (closest.isPresent()) {
			return closest.get().getKey();
		} else
			return null;
	}

	/**
	 * Determines the ID of the {@code AprilTag} that is closest (in terms of
	 * angular displacement) to the specified {@code Pose2d} ({@code null} if no
	 * such {@code AprilTag}).
	 * 
	 * @param pose a {@code Pose2d}
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @param angleOfCoverage The angle of coverage (in degrees) TODO
	 * @return the ID of the {@code AprilTag} that is closest (in terms of angular
	 *         displacement) to the specified {@code Pose2d} ({@code null} if no
	 *         such {@code AprilTag})
	 */
	public static Integer angleClosestTagID(Pose2d pose, double distanceThresholdInMeters,
			Optional<Double> angleOfCoverage) {
		var s = kFieldLayout.getTags().stream();
		if (angleOfCoverage.isPresent()) {
			s.filter(
					tag -> Math
							.abs(angularDisplacement(pose, tag.pose.toPose2d()).getDegrees()) < angleOfCoverage
									.get());
		} else {
			s.filter(tag -> Math.abs(angularDisplacement(pose, tag.pose.toPose2d()).getDegrees()) < 90);
		}
		Optional<Entry<Integer, Double>> closest = s
				.map(tag -> Map.entry(tag.ID, Math.abs(translationalDisplacement(pose, tag.pose.toPose2d()))))
				.filter(tag -> tag.getValue() < distanceThresholdInMeters)
				.reduce((tag1, tag2) -> tag1.getValue() < tag2.getValue() ? tag1 : tag2);
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
		var translation = last.getTranslation().minus(initial.getTranslation());
		return Math.abs(angularDisplacement(initial, last).getDegrees()) > 90 ? -translation.getNorm()
				: translation.getNorm();
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
		var translation = last.getTranslation().minus(initial.getTranslation());
		return translation.getAngle().minus(initial.getRotation());
	}

	/**
	 * Constructs a {@code Translation2d}.
	 * 
	 * @param x the x component of the {@code Translation2d}
	 * @param y the y component of the {@code Translation2d}
	 * @return the constructed {@code Translation2d}
	 */
	public static Translation2d translation(double x, double y) {
		return new Translation2d(x, y);
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
	 * Constructs a {@code Transform2d}.
	 * 
	 * @param x the x component of the {@code Transform2d}
	 * @param y the y component of the {@code Transform2d}
	 * @param yawInDegrees the rotational component of the {@code Transform2d} in
	 *        degrees
	 * @return the constructed {@code Transform2d}
	 */
	public static Transform2d transform(double x, double y, double yawInDegrees) {
		return new Transform2d(x, y, Rotation2d.fromDegrees(yawInDegrees));
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