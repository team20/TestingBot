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

	public AutoRoutine leave() {
		AutoRoutine routine = m_factory.newRoutine("LeaveRoutine");

		AutoTrajectory trajectory = routine.trajectory("Leave");

		routine.active().onTrue(
				sequence(
						trajectory.resetOdometry(),
						print("Leaving"),
						trajectory.cmd()));

		return routine;
	}

	public AutoRoutine roundTrip() {
		AutoRoutine routine = m_factory.newRoutine("RoundTripRoutine");

		AutoTrajectory trajectory = routine.trajectory("Round Trip");

		routine.active().onTrue(
				sequence(
						trajectory.resetOdometry(),
						trajectory.cmd()));

		return routine;
	}

	public AutoRoutine middleBall() {
		AutoRoutine routine = m_factory.newRoutine("MiddleBallRoutine");

		AutoTrajectory trajectory = routine.trajectory("Middle Ball");

		routine.active().onTrue(
				sequence(
						trajectory.resetOdometry(),
						trajectory.cmd()));

		return routine;
	}

	public Supplier<Command> zigZag(double dist, int times) {
		return () -> {
			Command command = none();
			for (double i = 0, direction = 45; i < times; i++)
				command = command.andThen(m_driveSubsystem.driveForDistance(dist, direction *= -1));
			return command;
		};
	}
}
