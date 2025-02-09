package frc.robot.subsystems;

import static frc.robot.subsystems.PoseEstimationSubsystem.*;
import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;

public class PoseEstimationSubsystemTest {

	@BeforeAll
	static void setup() {
		assert HAL.initialize(500, 0);
	}

	@AfterEach
	void shutdown() throws Exception {
	}

	@Test
	void testCircleCenter() {
		var center = PoseEstimationSubsystem.circleCenter(translation(1, 0), translation(0, 1), translation(-1, 0));
		assertEquals(0, center.getX(), 1e-2);
		assertEquals(0, center.getY(), 1e-2);

		var r = new java.util.Random(31);
		for (int i = 0; i < 100; i++)
			testCircleCenter(
					r.nextDouble(), r.nextDouble(), r.nextDouble(), r.nextDouble() * 2 * Math.PI,
					r.nextDouble() * 2 * Math.PI, r.nextDouble() * 2 * Math.PI);
	}

	@Test
	void testInterpolate() {
		var p = interpolate(translation(1, 0), translation(-1, 0), translation(0, 0));
		assertEquals(0, p.getX(), 1e-2);
		assertEquals(1, p.getY(), 1e-2);

		p = interpolate(translation(1, 0), translation(0, -1), translation(0, 0));
		System.out.println(p);
		assertEquals(Math.cos(-Math.PI / 4), p.getX(), 1e-2);
		assertEquals(Math.sin(-Math.PI / 4), p.getY(), 1e-2);

		var r = new java.util.Random(31);
		for (int i = 0; i < 100; i++)
			testInterpolate(
					r.nextDouble(), r.nextDouble(), r.nextDouble(), r.nextDouble() * 2 * Math.PI,
					r.nextDouble() * Math.PI / 2 - Math.PI / 4);
	}

	@Test
	void testRefine() {
		var path = refine(pose(1, 0, 0), pose(0, 1, 90), pose(-1, 0, 180));
		assertEquals(Math.cos(Math.PI / 4), path[1].getX(), 1e-2);
		assertEquals(Math.sin(Math.PI / 4), path[1].getY(), 1e-2);

		assertEquals(Math.cos(Math.PI / 4 * 3), path[3].getX(), 1e-2);
		assertEquals(Math.sin(Math.PI / 4 * 3), path[3].getY(), 1e-2);
	}

	void testCircleCenter(double x, double y, double r, double a1, double a2, double a3) {
		var center = translation(x, y);
		var p1 = translation(r * Math.cos(a1), r * Math.sin(a1)).plus(center);
		var p2 = translation(r * Math.cos(a2), r * Math.sin(a2)).plus(center);
		var p3 = translation(r * Math.cos(a3), r * Math.sin(a3)).plus(center);
		center = circleCenter(p1, p2, p3);
		// System.out.println(center + " <- " + p1 + " " + p2 + " " + p3);
		assertEquals(x, center.getX(), 1e-2);
		assertEquals(y, center.getY(), 1e-2);
	}

	void testInterpolate(double x, double y, double r, double a1, double a2) {
		var center = translation(x, y);
		var p1 = translation(r * Math.cos(a1), r * Math.sin(a1)).plus(center);
		var p2 = translation(r * Math.cos(a1 + a2), r * Math.sin(a1 + a2)).plus(center);
		var p3 = translation(r * Math.cos(a1 + 2 * a2), r * Math.sin(a1 + 2 * a2)).plus(center);
		var p = interpolate(p1, p3, center);
		assertEquals(p2.getX(), p.getX(), 1e-2);
		assertEquals(p2.getY(), p.getY(), 1e-2);
	}

}
