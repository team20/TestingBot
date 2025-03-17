// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.DriveConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Contains all the hardware and controllers for a swerve module.
 */
public class SwerveModule {
	private final PIDController m_steerController = new PIDController(kP, kI, kD);
	protected final CANcoder m_CANCoder;
	protected final SparkMax m_driveMotor;
	protected final SparkMax m_steerMotor;
	private final SparkMaxSim m_driveMotorSim;
	private final SparkMaxSim m_steerMotorSim;
	private final DCMotorSim m_driveMotorModel;
	private final DCMotorSim m_steerMotorModel;

	public SwerveModule(int canId, int drivePort, int steerPort) {
		m_CANCoder = new CANcoder(canId);
		m_driveMotor = new SparkMax(drivePort, MotorType.kBrushless);
		m_steerMotor = new SparkMax(steerPort, MotorType.kBrushless);

		var config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake).voltageCompensation(12);
		config.openLoopRampRate(kRampRate).closedLoopRampRate(kRampRate);
		// Helps with encoder precision (not set in stone)
		config.encoder.uvwAverageDepth(kEncoderDepth).uvwMeasurementPeriod(kEncoderMeasurementPeriod);
		config.smartCurrentLimit(kDriveSmartCurrentLimit).secondaryCurrentLimit(kDrivePeakCurrentLimit);
		m_driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		config = new SparkMaxConfig();
		config.idleMode(IdleMode.kBrake).voltageCompensation(12);
		config.openLoopRampRate(kRampRate).closedLoopRampRate(kRampRate);
		// Helps with encoder precision (not set in stone)
		config.encoder.uvwAverageDepth(kEncoderDepth).uvwMeasurementPeriod(kEncoderMeasurementPeriod);
		config.smartCurrentLimit(kSteerSmartCurrentLimit).secondaryCurrentLimit(kSteerSecondaryCurrentLimit);
		m_steerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

		m_steerController.enableContinuousInput(0, 360);
		if (RobotBase.isSimulation()) {
			m_driveMotorSim = new SparkMaxSim(m_driveMotor, DCMotor.getNEO(1));
			m_steerMotorSim = new SparkMaxSim(m_steerMotor, DCMotor.getNEO(1));
			m_driveMotorModel = new DCMotorSim(
					LinearSystemId.createDCMotorSystem(kV / (2 * Math.PI), kA / (2 * Math.PI)),
					DCMotor.getKrakenX60(1).withReduction(kDriveGearRatio));
			m_steerMotorModel = new DCMotorSim(
					LinearSystemId.createDCMotorSystem(kV / (2 * Math.PI), kA / (2 * Math.PI)),
					DCMotor.getKrakenX60(1));
		} else {
			m_driveMotorSim = null;
			m_steerMotorSim = null;
			m_driveMotorModel = null;
			m_steerMotorModel = null;
		}
	}

	/**
	 * Returns the drive encoder distance in meters.
	 * 
	 * @return the drive encoder position in meters
	 */
	public double getDriveEncoderPosition() {
		return m_driveMotor.getEncoder().getPosition() * kMetersPerMotorRotation;
	}

	/**
	 * Returns the steer current of this {@code SwerveModule}.
	 * 
	 * @return the steer current of this {@code SwerveModule}
	 */
	public double getSteerCurrent() {
		return m_steerMotor.getOutputCurrent();
	}

	/**
	 * Returns the drive current of this {@code SwerveModule}.
	 * 
	 * @return the drive current of this {@code SwerveModule}
	 */
	public double getDriveCurrent() {
		return m_driveMotor.getOutputCurrent();
	}

	/**
	 * Resets the drive encoder to zero.
	 */
	public void resetDriveEncoder() {
		m_driveMotor.getEncoder().setPosition(0);
	}

	/**
	 * Returns the current drive motor voltage.
	 * 
	 * @return the motor speed in voltage
	 */
	public double getDriveVoltage() {
		return m_driveMotor.getAppliedOutput() * kDriveMaxVoltage;
	}

	/**
	 * Returns the current drive motor temperature.
	 * 
	 * @return the temperature in degrees Celsius
	 */
	public double getDriveTemperature() {
		return m_driveMotor.getMotorTemperature();
	}

	/**
	 * Returns the module angle in degrees.
	 * 
	 * @return the module angle in degrees
	 */
	public double getModuleAngle() {
		return m_CANCoder.getAbsolutePosition().getValueAsDouble() * 360;
	}

	/**
	 * Returns the current {@code SwerveModulePosition} of this
	 * {@code SwerveModule}.
	 * 
	 * @return the current {@code SwerveModulePosition} of this {@code SwerveModule}
	 */
	public SwerveModulePosition getModulePosition() {
		return new SwerveModulePosition(getDriveEncoderPosition(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Returns the current {@code SwerveModuleState} of this {@code SwerveModule}.
	 * 
	 * @return the current {@code SwerveModuleState} of this {@code SwerveModule}
	 */
	public SwerveModuleState getModuleState() {
		return new SwerveModuleState(getDriveVoltage(), Rotation2d.fromDegrees(getModuleAngle()));
	}

	/**
	 * Sets the drive motor speeds and module angle of this {@code SwerveModule}.
	 * 
	 * @param state a {@code SwerveModuleState} containing the target speeds and
	 *        angle
	 */
	public void setModuleState(SwerveModuleState state) {
		m_driveMotor.setVoltage(state.speedMetersPerSecond);
		setAngle(state.angle.getDegrees());
	}

	/**
	 * Sets the module angle of this {@code SwerveModule}.
	 * 
	 * @param angle the target angle in degrees
	 */
	public void setAngle(double angle) {
		m_steerMotor.setVoltage(m_steerController.calculate(getModuleAngle(), angle));
		if (RobotBase.isSimulation())
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