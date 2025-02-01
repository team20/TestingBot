package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.util.function.Supplier;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class CommandComposer {
	private final DriveSubsystem m_driveSubsystem;
	private final AutoFactory m_factory;

	public CommandComposer(DriveSubsystem driveSubsystem) {
		m_driveSubsystem = driveSubsystem;

		m_factory = new AutoFactory(
				m_driveSubsystem::getPose,
				m_driveSubsystem::resetOdometry,
				m_driveSubsystem::trajectoryFollower,
				true,
				m_driveSubsystem);
	}

	// LEAVE AUTOS:

	public AutoRoutine blue1Leave() {
		AutoRoutine routine = m_factory.newRoutine("Blue Leave 1");
		AutoTrajectory trajectory = routine.trajectory("LeaveB1");
		routine.active().onTrue(
				sequence(
						trajectory.resetOdometry(),
						print("Leaving B1"),
						trajectory.cmd()));
		return routine;
	}

	public AutoRoutine blue5Leave() {
		AutoRoutine routine = m_factory.newRoutine("Blue Leave 5");
		AutoTrajectory trajectory = routine.trajectory("LeaveB5");
		routine.active().onTrue(
				sequence(
						trajectory.resetOdometry(),
						print("Leaving B5"),
						trajectory.cmd()));
		return routine;
	}

	public AutoRoutine red1Leave() {
		AutoRoutine routine = m_factory.newRoutine("Red Leave 1");
		AutoTrajectory trajectory = routine.trajectory("LeaveR1");
		routine.active().onTrue(
				sequence(
						trajectory.resetOdometry(),
						print("Leaving R1"),
						trajectory.cmd()));
		return routine;
	}

	public AutoRoutine red5Leave() {
		AutoRoutine routine = m_factory.newRoutine("Red Leave 1");
		AutoTrajectory trajectory = routine.trajectory("LeaveR1");
		routine.active().onTrue(
				sequence(
						trajectory.resetOdometry(),
						print("Leaving R1"),
						trajectory.cmd()));
		return routine;
	}

	// REEF SCORE AUTOS

	// TESTING AUTOS:

	public Supplier<Command> zigZag(double dist, int times) {
		return () -> {
			Command command = none();
			for (double i = 0, direction = 45; i < times; i++)
				command = command.andThen(m_driveSubsystem.driveForDistance(dist, direction *= -1));
			return command;
		};
	}
}
