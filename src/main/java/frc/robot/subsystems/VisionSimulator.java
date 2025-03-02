package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.LinkedList;
import java.util.Optional;
import java.util.Random;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A {@code VisionSimulator} aims to provide realistic simulations of vision on
 * the field.
 */
public class VisionSimulator extends SubsystemBase {
	private final DriveSubsystem m_driveSubsystem;
	private final VisionSystemSim m_visionSimulator;
    private final PhotonCameraSim m_cameraSim;
    private final SimCameraProperties simProp;
	private Pose2d m_previousOdometryPose = null;
	private double m_measurementErrorRatio;
    private final StructPublisher<Pose3d> m_cameraPosePublisher;
	private final StructPublisher<Pose2d> m_posePublisher;

	/**
	 * Constructs a {@code VisionSimulator}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to be used by the
	 *        {@code VisionSimulator}
	 * @param initialPose the initial {@code Pose2d} of the robot in simulation
	 * @param measurementErrorRatio the error ratio in measurements for
	 *        updating the odometry of the robot
	 */
	public VisionSimulator(String cameraName, Transform3d robotToCamera, DriveSubsystem driveSubsystem, Pose2d initialPose, double measurementErrorRatio) {
		m_driveSubsystem = driveSubsystem;
		m_visionSimulator = new VisionSystemSim("Simulator");
		m_visionSimulator.addCamera(m_cameraSim, null);
        m_visionSimulator.addAprilTags(kFieldLayout);
        simProp = new SimCameraProperties();
        simProp.setCalibError(measurementErrorRatio, measurementErrorRatio);
		m_visionSimulator.resetRobotPose(initialPose);
		m_measurementErrorRatio = measurementErrorRatio;
		m_posePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@Simulation", Pose2d.struct)
				.publish();
        m_cameraPosePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/" + cameraName + "@Simulation", Pose3d.struct)
				.publish();
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}.
	 */
	@Override
	public void periodic() {
		var pose1 = m_driveSubsystem.getPose();
		var pose2 = m_visionSimulator.getRobotPose().toPose2d();
		if (m_previousOdometryPose != null) {
			var odometryDiff = pose1.minus(m_previousOdometryPose);
			odometryDiff = new Transform2d(change(odometryDiff.getX(), m_measurementErrorRatio),
					change(odometryDiff.getY(), m_measurementErrorRatio),
					Rotation2d.fromDegrees(change(odometryDiff.getRotation().getDegrees(), m_measurementErrorRatio)));
			pose2 = pose2.plus(odometryDiff);
		}
		m_previousOdometryPose = pose1;
		m_visionSimulator.update(pose2);
		m_posePublisher.set(pose2);
	}

    	/**
	 * Adds the specified {@code PhotonCameraSim} to this {@code VisionSimulator}.
	 * 
	 * @param cameraSim a {@code PhotonCameraSim}
	 * @param robotToCamera the {@code Transform3d} expressing the pose of the
	 *        camera relative to the pose of the robot.
	 */
	public void addCamera(PhotonCameraSim cameraSim, Transform3d robotToCamera) {
		m_visionSimulator.addCamera(cameraSim, robotToCamera);
	}

	/**
	 * Returns the {@code Pose3d} of the specified {@code PhotonCameraSim}.
	 * 
	 * @param m_cameraSim a {@code PhotonCameraSim}
	 * @return the {@code Pose3d} of the specified {@code PhotonCameraSim}
	 */
	public Optional<Pose3d> getCameraPose(PhotonCameraSim m_cameraSim) {
		return m_visionSimulator.getCameraPose(m_cameraSim);
	}

	/**
	 * Returns the most recent simulated {@code Pose2d} of the robot on the field.
	 * 
	 * @return the most recent simulated {@code Pose2d} of the robot on the field
	 */
	public Pose2d getSimulatedPose() {
		return m_visionSimulator.getRobotPose().toPose2d();
	}
}