package frc.robot.subsystems;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;

import edu.wpi.first.hal.HAL;

public class PoseEstimationSubsystemTest {

	@BeforeAll
	static void setup() {
		assert HAL.initialize(500, 0);
	}

	@AfterEach
	void shutdown() throws Exception {
	}

}
