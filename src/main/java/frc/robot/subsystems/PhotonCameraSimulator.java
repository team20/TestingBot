package frc.robot.subsystems;

import static frc.robot.Constants.kFieldLayout;
import static frc.robot.Constants.VisionConstants.kAvgLatencyMs;
import static frc.robot.Constants.VisionConstants.kLatencyStdDevMs;
import static frc.robot.Constants.VisionConstants.kSimAvgError;
import static frc.robot.Constants.VisionConstants.kSimStdDevError;

import java.util.LinkedList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * A {@code PhotonCameraSimulator} aims to provide realistic simulations of a
 * {@code PhotonCamera} with artificially injected auglar errors and delays.
 */
public class PhotonCameraSimulator extends PhotonCamera {
	private PhotonCameraSim m_cameraSim = null;
	private final LinkedList<PhotonPipelineResult> m_buffered = new LinkedList<PhotonPipelineResult>();
	private final LinkedList<PhotonPipelineResult> m_unreadResults = new LinkedList<PhotonPipelineResult>();
	private PhotonPipelineResult m_latestResult = new PhotonPipelineResult();
	private final StructPublisher<Pose3d> m_cameraPosePublisher;
	
	private final VisionSystemSim m_visionSimulator;
	private final SimCameraProperties m_simProp;
	private Pose2d m_previousOdometryPose = null;
	private final StructPublisher<Pose2d> m_posePublisher;

	/**
	 * Constructs a {@code PhotonCameraSimulator}.
	 * 
	 * @param cameraName the nickname of the camera to be used by the
	 *        {@code PhotonCameraSimulator} (found in the PhotonVision UI)
	 * @param robotToCamera the {@code Transform3d} expressing the pose of the
	 *        camera relative to the pose of the robot.
	 * @param visionSimulator the {@code VisionSimulator} to be used by the
	 *        {@code PhotonCameraSimulator}
	 * @param initialPose the initial {@code Pose2d} of the robot in simulation
	 * @param measurmentErrorRatio the error ratio in measurements for
	 *        updating the odometry of the robot
	 * @param maxAngularErrorInDegrees the maximum artificial angular error to
	 *        inject in degrees
	 * @param delayInSeconds the artificial delay to add in seconds
	 */
	public PhotonCameraSimulator(String cameraName, Transform3d robotToCamera) {
		super(cameraName);
		m_visionSimulator = new VisionSystemSim("Simulator");
		m_visionSimulator.addAprilTags(kFieldLayout);
		m_simProp = new SimCameraProperties();
		m_simProp.setCalibError(kSimAvgError, kSimStdDevError);
		m_simProp.setAvgLatencyMs(kAvgLatencyMs);
		m_simProp.setLatencyStdDevMs(kLatencyStdDevMs);
		m_posePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@Simulation", Pose2d.struct)
				.publish();
		
		m_cameraSim = new PhotonCameraSim(this, m_simProp);
		m_cameraSim.enableProcessedStream(true);
		m_cameraSim.enableDrawWireframe(true);
		m_visionSimulator.addCamera(m_cameraSim, robotToCamera);
		m_cameraPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/" + cameraName + "@Simulation", Pose3d.struct)
				.publish();
	}

	public void addCamera(PhotonCamera camera, Transform3d robotToCamera) {
		m_visionSimulator.addCamera(new PhotonCameraSim(camera), robotToCamera);
	}

	public void update(Pose2d robotPose) {
		m_visionSimulator.update(robotPose);
	}
}