package frc.robot.subsystems;

import static frc.robot.Constants.*;

import java.util.Optional;
import java.util.Random;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

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

	/**
	 * The {@code DriveSubsystem} used by this {@code VisionSimulator}.
	 */
	private final DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code VisionSystemSim} used by this {@code VisionSimulator}.
	 */
	private final VisionSystemSim m_visionSimulator;

	/**
	 * The previous {@code Pose2d} of the robot from the {@code VisionSimulator}.
	 */
	private Pose2d m_previousOdometryPose = null;

	/**
	 * The error ratio in measurements for updating the odometry of the robot.
	 */
	private double m_measurementErrorRatio;

	/**
	 * The {@code StructPublisher} for reporting the {@code Pose2d} of the
	 * robot in simulation.
	 */
	private final StructPublisher<Pose2d> m_posePublisher;

	/**
	 * The {@code Random} instance used by this {@code VisionSimulator}.
	 */
	private final Random random = new Random(31);

	/**
	 * Constructs a {@code VisionSimulator}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to be used by the
	 *        {@code VisionSimulator}
	 * @param initialPose the initial {@code Pose2d} of the robot in simulation
	 * @param measurmentErrorRatio the error ratio in measurements for
	 *        updating the odometry of the robot
	 */
	public VisionSimulator(DriveSubsystem driveSubsystem, Pose2d initialPose, double measurmentErrorRatio) {
		m_driveSubsystem = driveSubsystem;
		m_visionSimulator = new VisionSystemSim("Simulator");
		m_visionSimulator.addAprilTags(kFieldLayout);
		m_visionSimulator.resetRobotPose(initialPose);
		m_measurementErrorRatio = measurmentErrorRatio;
		m_posePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@Simulation", Pose2d.struct)
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
	 * Returns the most recent simulated {@code Pose2d} of the robot on the field.
	 * 
	 * @return the most recent simulated {@code Pose2d} of the robot on the field
	 */
	public Pose2d getSimulatedPose() {
		return m_visionSimulator.getRobotPose().toPose2d();
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
	 * Randomly changes the specified value using the specified ratio.
	 * 
	 * @param x a value
	 * @param ratio the ratio by which the value can be increased or decreased
	 * @return the resulting value
	 */
	private double change(double x, double ratio) {
		return x * (1 - ratio + 2 * ratio * random.nextDouble());
	}

}