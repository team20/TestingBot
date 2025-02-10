package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;

import java.util.ArrayList;
import java.util.Arrays;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

	/**
	 * The {@code PhotonCamera}s and {@code PhotonPoseEstimator}s used by this
	 * {@code PoseEstimationSubsystem}.
	 */
	private final Map<PhotonCamera, PhotonPoseEstimator> m_cameras = new HashMap<PhotonCamera, PhotonPoseEstimator>();

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
	 * The {@code PIDController} for controlling the robot in the x and y
	 * dimensions in meters (input: error in meters, output: velocity in meters per
	 * second).
	 */
	private PIDController m_controllerXY;

	/**
	 * The {@code PIDController} for controlling the robot in the yaw
	 * dimension in radians (input: error in radians, output: velocity in radians
	 * per second).
	 */
	private PIDController m_controllerYaw;

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
				stateStdDevs,
				visionMeasurementStdDevs);
		m_detectedPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PhotonPoseEstimator", Pose2d.struct)
				.publish();
		m_estimatedPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@PoseEstimationSubsystem", Pose2d.struct)
				.publish();
		m_controllerXY = new PIDController(kDriveP, kDriveI, kDriveD);
		m_controllerYaw = new PIDController(kTurnP, kTurnI, kTurnD);
		m_controllerYaw.enableContinuousInput(0, 2 * Math.PI);
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
		for (var e : m_cameras.entrySet()) {
			var camera = e.getKey();
			var poseEstimator = e.getValue();
			for (var r : camera.getAllUnreadResults()) { // for every result r
				if (firstCamera || r.getTargets().size() > 1) {
					Optional<EstimatedRobotPose> p = poseEstimator.update(r);
					if (p.isPresent()) { // if successful
						EstimatedRobotPose v = p.get(); // get successfully estimated pose
						m_poseEstimator.addVisionMeasurement(v.estimatedPose.toPose2d(), v.timestampSeconds);
						m_detectedPosePublisher.set(v.estimatedPose.toPose2d());
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
	 * Calculates the {@code ChassisSpeeds} to move the robot toward the closest
	 * {@code AprilTag}.
	 * 
	 * @param robotToTag the {@code Tranform2d} representing the pose of the
	 *        {@code AprilTag} relative to the robot when the robot is aligned
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the calculated {@code ChassisSpeeds} to move the robot toward the
	 *         closest {@code AprilTag}
	 */
	public ChassisSpeeds chassisSpeedsTowardClosestTag(Transform2d robotToTag, double distanceThresholdInMeters) {
		var currentRobotPose = getEstimatedPose();
		var closestTagPose = closestTagPose(180, distanceThresholdInMeters);
		if (closestTagPose == null)
			return new ChassisSpeeds();
		var targetRobotPose = closestTagPose.plus(robotToTag);
		return chassisSpeeds(currentRobotPose, targetRobotPose, m_controllerXY, m_controllerYaw);
	}

	/**
	 * Calculates the {@code ChassisSpeeds} to move from the current {@code Pose2d}
	 * toward the target {@code Pose2d}.
	 * 
	 * @param currentPose the current {@code Pose2d}
	 * @param targetPose the target {@code Pose2d}
	 * @param controllerXY the {@code PIDController} for controlling the robot in
	 *        the x and y dimensions in meters (input: error in meters, output:
	 *        velocity in meters per second)
	 * @param controllerYaw the {@code PIDController} for controlling the robot in
	 *        the yaw dimension in radians (input: error in radians, output:
	 *        velocity in radians per second)
	 * @return the calculated {@code ChassisSpeeds} to move from the current
	 *         {@code Pose2d} toward the target {@code Pose2d}
	 */
	public ChassisSpeeds chassisSpeeds(Pose2d currentPose, Pose2d targetPose, PIDController controllerXY,
			PIDController controllerYaw) {
		Translation2d current2target = targetPose.minus(currentPose).getTranslation()
				.rotateBy(m_driveSubsystem.getPose().getRotation());
		double velocityX = 0, velocityY = 0;
		double distance = current2target.getNorm();
		if (distance > 0) {
			double speed = Math.abs(controllerXY.calculate(distance, 0));
			velocityX = speed * current2target.getAngle().getCos();
			velocityY = speed * current2target.getAngle().getSin();
		}
		double angularVelocityRadiansPerSecond = controllerYaw
				.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
		return new ChassisSpeeds(velocityX, velocityY, angularVelocityRadiansPerSecond);
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
	 * Finds the {@code Pose2d} of the {@code AprilTag} that is closest to the robot
	 * ({@code null} if no such {@code AprilTag}).
	 * 
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
	 * Finds the {@code Pose2d} of the {@code AprilTag} that is closest (in terms of
	 * angular displacement) to the robot when the specified {@code Translation2d}
	 * is applied to the robot.
	 * 
	 * @param translation a {@code Translation2d}
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the {@code Pose2d} of the {@code AprilTag} that is closest (in terms
	 *         of angular displacement) to the robot when the specified
	 *         {@code Translation2d} is applied to the robot
	 */
	public Pose2d closestTagPose(Translation2d translation, double distanceThresholdInMeters) {
		var pose = getEstimatedPose();
		if (translation.getNorm() > 0)
			pose = new Pose2d(pose.getTranslation(), translation.getAngle());
		var i = closestTagID(pose, distanceThresholdInMeters);
		return i == null ? null : kFieldLayout.getTagPose(i).get().toPose2d();
	}

	/**
	 * Determines the ID of the {@code AprilTag} that is closest (in terms of
	 * angular displacement) to the specified {@code Pose2d} ({@code null} if no
	 * such {@code AprilTag}).
	 * 
	 * @param pose a {@code Pose2d}
	 * @param distanceThresholdInMeters the maximum distance (in meters) within
	 *        which {@code AprilTag}s are considered
	 * @return the ID of the {@code AprilTag} that is closest (in terms of angular
	 *         displacement) to the specified {@code Pose2d} ({@code null} if no
	 *         such {@code AprilTag})
	 */
	public static Integer closestTagID(Pose2d pose, double distanceThresholdInMeters) {
		var s = kFieldLayout.getTags().stream()
				// consider only the tags facing toward the robot
				.filter(
						t -> Math.abs(
								t.pose.getTranslation().toTranslation2d().minus(pose.getTranslation()).getAngle()
										.minus(t.pose.toPose2d().getRotation()).getDegrees()) > 90)
				.filter(t -> Math.abs(translationalDisplacement(pose, t.pose.toPose2d())) < distanceThresholdInMeters)
				// only tags sufficently close
				.map(t -> Map.entry(t.ID, Math.abs(angularDisplacement(pose, t.pose.toPose2d()).getDegrees())));
		Optional<Entry<Integer, Double>> closest = s.reduce((e1, e2) -> e1.getValue() < e2.getValue() ? e1 : e2);
		if (closest.isPresent()) {
			return closest.get().getKey();
		} else
			return null;
	}

	/**
	 * Determines the ID of the {@code AprilTag} that is closest to the robot
	 * ({@code null} if no such {@code AprilTag}).
	 * 
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
	 * Calculates the angular displacement from the estimated {@code Pose2d} of the
	 * robot to the specified {@code AprilTag}.
	 * 
	 * @param tagID the ID of the {@code AprilTag}
	 * @return the angular displacement from the estimated {@code Pose2d} of the
	 *         robot to the specified {@code AprilTag}
	 */
	public Rotation2d angularDisplacement(int tagID) {
		try {
			return angularDisplacement(this.getEstimatedPose(), kFieldLayout.getTagPose(tagID).get().toPose2d());
		} catch (Exception e) {
			return null;
		}
	}

	/**
	 * >>>>>>> auto-align
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
		return Math.abs(t.getAngle().minus(initial.getRotation()).getDegrees()) > 90 ? -t.getNorm() : t.getNorm();
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

	/**
	 * Returns the {@code Pose2d}s of the specified {@code AprilTag}s.
	 * 
	 * @param tagIDs the IDs of the {@code AprilTag}s
	 * @return the {@code Pose2d}s of the specified {@code AprilTag}s
	 */
	public static Pose2d[] tagPoses(int... tagIDs) {
		return Arrays.stream(tagIDs).mapToObj(i -> kFieldLayout.getTagPose(i))
				.filter(p -> p.isPresent())
				.map(p -> p.get())
				.map(p -> p.toPose2d()).toList().toArray(new Pose2d[0]);
	}

	/**
	 * Adds intermediate {@code Pose2d}s to the specified path to make the path
	 * smoother.
	 * 
	 * @param path the {@code Pose2d}s constituting the path
	 * @return the {@code Pose2d}s in the resulting (i.e., refined) path
	 */
	public static Pose2d[] refine(Pose2d... path) {
		var l = new ArrayList<Pose2d>();
		for (int i = 0; i < path.length - 1; i++)
			l.add(
					intermediate(
							i - 1 < 0 ? null : path[i - 1], path[i], path[i + 1],
							i + 2 >= path.length ? null : path[i + 2]));
		for (int i = 0; i < path.length; i++)
			l.add(2 * i, path[i]);
		return l.toArray(new Pose2d[0]);
	}

	/**
	 * Calculates an intermediate {@code Pose2d} given the specified
	 * {@code Pose2d}s.
	 * 
	 * @param previousPrevious the {@code Pose2d} right before the previous
	 *        {@code Pose2d}
	 * @param previous the previous {@code Pose2d}
	 * @param next the next {@code Pose2d}
	 * @param nextNext the {@code Pose2d} right after the next {@code Pose2d}
	 * @return
	 */
	private static Pose2d intermediate(Pose2d previousPrevious, Pose2d previous, Pose2d next, Pose2d nextNext) {
		var t = previous.getTranslation().plus(next.getTranslation()).div(2);
		Translation2d cPrevious = previousPrevious == null ? null
				: circleCenter(
						previousPrevious.getTranslation(), previous.getTranslation(), next.getTranslation());
		Translation2d cNext = nextNext == null ? null
				: circleCenter(
						previous.getTranslation(), next.getTranslation(), nextNext.getTranslation());
		if (cPrevious != null && cNext != null)
			t = interpolate(previous.getTranslation(), next.getTranslation(), cPrevious)
					.plus(interpolate(previous.getTranslation(), next.getTranslation(), cNext)).div(2);
		else if (cPrevious != null && cNext == null)
			t = interpolate(previous.getTranslation(), next.getTranslation(), cPrevious);
		else if (cPrevious == null && cNext != null)
			t = interpolate(previous.getTranslation(), next.getTranslation(), cNext);
		var r = mean(previous.getRotation(), next.getRotation());
		return new Pose2d(t, r);
	}

	/**
	 * Returns the center of the circle that passes through the specified
	 * {@code Translation2d}s.
	 * 
	 * @param p1 a {@code Translation2d}
	 * @param p2 a {@code Translation2d}
	 * @param p3 a {@code Translation2d}
	 * @return the center of the circle that passes through the specified
	 *         {@code Translation2d}s
	 */
	public static Translation2d circleCenter(Translation2d p1, Translation2d p2, Translation2d p3) {
		double x1 = p1.getX();
		double y1 = p1.getY();
		double x2 = p2.getX();
		double y2 = p2.getY();
		double x3 = p3.getX();
		double y3 = p3.getY();
		double d = 2.0 * (x1 * (y2 - y3)
				+ x2 * (y3 - y1)
				+ x3 * (y1 - y2));
		if (Math.abs(d) < 1e-12)
			return null;
		double x = ((x1 * x1 + y1 * y1) * (y2 - y3)
				+ (x2 * x2 + y2 * y2) * (y3 - y1)
				+ (x3 * x3 + y3 * y3) * (y1 - y2)) / d;
		double y = ((x1 * x1 + y1 * y1) * (x3 - x2)
				+ (x2 * x2 + y2 * y2) * (x1 - x3)
				+ (x3 * x3 + y3 * y3) * (x2 - x1)) / d;
		return new Translation2d(x, y);
	}

	/**
	 * Returns the middle {@code Translation2d} between the two specified
	 * {@code Translation2d}s on the circle centered at the specified
	 * {@code Translation2d}.
	 * 
	 * @param p1 a {@code Translation2d}
	 * @param p2 a {@code Translation2d}
	 * @param center the center of the circle that passes through the specified
	 *        {@code Translation2d}s
	 * @return the middle {@code Translation2d} between the two specified
	 *         {@code Translation2d}s on the circle centered at the specified
	 *         {@code Translation2d}
	 */
	public static Translation2d interpolate(Translation2d p1, Translation2d p2, Translation2d center) {
		if (center == null)
			return p1.plus(p2).div(2);
		p1 = p1.minus(center);
		p2 = p2.minus(center);
		double r = (p1.getNorm() + p2.getNorm()) / 2;
		var angle = mean(p1.getAngle(), p2.getAngle());
		return translation(r, 0).rotateBy(angle).plus(center);
	}

	/**
	 * Returns the mean of the two specified {@code Rotation2d}s.
	 * 
	 * @param r1 a {@code Rotation2d}
	 * @param r2 a {@code Rotation2d}
	 * @return the mean of the two specified {@code Rotation2d}s
	 */
	private static Rotation2d mean(Rotation2d r1, Rotation2d r2) {
		return r2.minus(r1).div(2).plus(r1);
	}

}