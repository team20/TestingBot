package frc.robot.subsystems;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotConstants.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
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
	public Pose2d getPose() {
		return m_poseEstimator.getEstimatedPosition();
	}
}