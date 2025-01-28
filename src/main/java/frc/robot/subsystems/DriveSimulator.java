package frc.robot.subsystems;

import java.util.Random;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A {@code DriveSimulator} aims to provide realistic simulations of the
 * {@code Pose2d} of the robot on the field.
 */
public class DriveSimulator extends SubsystemBase {

	/**
	 * The {@code DriveSubsystem} used by this {@code DriveSimulator}.
	 */
	private final DriveSubsystem m_driveSubsystem;

	/**
	 * The {@code Pose2d} of the robot in simulation.
	 */
	private Pose2d m_pose;

	/**
	 * The previous {@code Pose2d} of the robot from the {@code DriveSubsystem}.
	 */
	private Pose2d m_previousOdometryPose = null;

	/**
	 * The error ratio in measurements for updating the odometry of the robot.
	 */
	private double m_measurmentErrorRatio;

	/**
	 * The {@code StructPublisher} for reporting the {@code Pose2d} of the
	 * robot in simulation.
	 */
	private final StructPublisher<Pose2d> m_posePublisher;

	/**
	 * The {@code Random} instance used by this {@code DriveSimulator}.
	 */
	private final Random random = new Random(31);

	/**
	 * Constructs a {@code DriveSimulator}.
	 * 
	 * @param driveSubsystem the {@code DriveSubsystem} to be used by the
	 *        {@code PhotonCameraSimulator}
	 * @param initialPose the initial {@code Pose2d} of the robot in simulation
	 * @param measurmentErrorRatio the error ratio in measurements for
	 *        updating the odometry of the robot
	 */
	public DriveSimulator(DriveSubsystem driveSubsystem, Pose2d initialPose, double measurmentErrorRatio) {
		m_driveSubsystem = driveSubsystem;
		m_pose = initialPose;
		m_measurmentErrorRatio = measurmentErrorRatio;
		m_posePublisher = NetworkTableInstance.getDefault()
				.getStructTopic("/SmartDashboard/Pose@Simulation", Pose2d.struct)
				.publish();
	}

	/**
	 * Is invoked periodically by the {@link CommandScheduler}.
	 */
	@Override
	public void periodic() {
		var p = m_driveSubsystem.getPose();
		if (m_previousOdometryPose != null) {
			var t = p.minus(m_previousOdometryPose);
			t = new Transform2d(change(t.getX(), m_measurmentErrorRatio), change(t.getY(), m_measurmentErrorRatio),
					Rotation2d.fromDegrees(change(t.getRotation().getDegrees(), m_measurmentErrorRatio)));
			m_pose = m_pose.plus(t);
		}
		m_previousOdometryPose = p;
		m_posePublisher.set(m_pose);
	}

	/**
	 * Returns the most recent simulated {@code Pose2d} of the robot on the field.
	 * 
	 * @return the most recent simulated {@code Pose2d} of the robot on the field
	 */
	public Pose2d getSimulatedPose() {
		return m_pose;
	}

	/**
	 * Randomly changes the specified value using the specified ratio.
	 * 
	 * @param x a value
	 * @param r the ratio by which the value can be increased or decreased
	 * @return the resulting value
	 */
	private double change(double x, double r) {
		return x * (1 - r + 2 * r * random.nextDouble());
	}

}