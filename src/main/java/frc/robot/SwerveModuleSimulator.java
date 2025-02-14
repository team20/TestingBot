// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.*;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Contains all the hardware and controllers for a swerve module.
 */
public class SwerveModuleSimulator extends SwerveModule {
	private final SparkMaxSim m_driveMotorSim;
	private final SparkMaxSim m_steerMotorSim;
	private final DCMotorSim m_driveMotorModel;
	private final DCMotorSim m_steerMotorModel;

	public SwerveModuleSimulator(int canId, int drivePort, int steerPort) {
		super(canId, drivePort, steerPort);
		m_driveMotorSim = new SparkMaxSim(m_driveMotor, DCMotor.getNEO(1));
		m_steerMotorSim = new SparkMaxSim(m_steerMotor, DCMotor.getNEO(1));
		m_driveMotorModel = new DCMotorSim(
				LinearSystemId.createDCMotorSystem(kV / (2 * Math.PI), kA / (2 * Math.PI)),
				DCMotor.getKrakenX60(1).withReduction(kDriveGearRatio));
		m_steerMotorModel = new DCMotorSim(
				LinearSystemId.createDCMotorSystem(kV / (2 * Math.PI), kA / (2 * Math.PI)),
				DCMotor.getKrakenX60(1));
	}

	/**
	 * Sets the drive motor speeds and module angle of this
	 * {@code SwerveModuleSimulator}.
	 * 
	 * @param state a {@code SwerveModuleState} containing the target speeds and
	 *        angle
	 */
	@Override
	public void setModuleState(SwerveModuleState state) {
		super.setModuleState(state);
		update();
	}

	/**
	 * Sets the module angle of this {@code SwerveModuleSimulator}.
	 * 
	 * @param angle the target angle in degrees
	 */
	@Override
	public void setAngle(double angle) {
		super.setAngle(angle);
		update();
	}

	/**
	 * Updates this {@code SwerveModuleSimulator}.
	 */
	private void update() {
		m_driveMotorModel.setInputVoltage(m_driveMotorSim.getAppliedOutput() * kDriveMaxVoltage);
		m_driveMotorModel.update(TimedRobot.kDefaultPeriod);
		m_driveMotorSim
				.iterate(m_driveMotorModel.getAngularVelocityRPM(), kDriveMaxVoltage, TimedRobot.kDefaultPeriod);
		m_driveMotorSim.setPosition(m_driveMotorModel.getAngularPositionRotations());
		m_driveMotorSim.setVelocity(m_driveMotorModel.getAngularVelocityRPM());

		m_steerMotorModel.setInputVoltage(m_steerMotorSim.getAppliedOutput() * kDriveMaxVoltage);
		m_steerMotorModel.update(TimedRobot.kDefaultPeriod);
		m_steerMotorSim
				.iterate(m_steerMotorModel.getAngularVelocityRPM(), kDriveMaxVoltage, TimedRobot.kDefaultPeriod);
		var encoderSimState = m_CANCoder.getSimState();
		encoderSimState.setRawPosition(m_steerMotorModel.getAngularPositionRotations() / kSteerGearRatio);
		encoderSimState.setVelocity(m_steerMotorModel.getAngularVelocityRPM() / kSteerGearRatio);
	}

}