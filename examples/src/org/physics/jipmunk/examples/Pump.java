package org.physics.jipmunk.examples;

import org.physics.jipmunk.*;
import org.physics.jipmunk.constraints.GearJoint;
import org.physics.jipmunk.constraints.PinJoint;
import org.physics.jipmunk.constraints.PivotJoint;
import org.physics.jipmunk.constraints.SimpleMotor;

import static org.physics.jipmunk.Util.cpv;
import static org.physics.jipmunk.Util.cpvzero;

/** @author jobernolte */
public class Pump extends ExampleBase {
	private Space space;
	private SimpleMotor motor;
	private final static float M_PI_2 = (float) (Math.PI / 2);
	private final static int numBalls = 5;
	private Body[] balls = new Body[numBalls];

	static Body addBall(Space space, Vector2f pos) {
		Body body = space.addBody(new Body(1.0f, Util.momentForCircle(1.0f, 30, 0, cpvzero())));
		body.setPosition(pos);

		Shape shape = space.addShape(new CircleShape(body, 30, cpvzero()));
		shape.setElasticity(0.0f);
		shape.setFriction(0.5f);

		return body;
	}

	@Override
	public Space init() {
		this.space = new Space();
		space.setGravity(cpv(0, -600));

		Body staticBody = space.getStaticBody();
		Shape shape;

		// beveling all of the line segments slightly helps prevent things from getting stuck on cracks
		shape = space.addShape(new SegmentShape(staticBody, cpv(-256, 16), cpv(-256, 300), 2.0f));
		shape.setElasticity(0.0f);
		shape.setFriction(0.5f);
		shape.setFilter(NOT_GRABABLE_FILTER);

		shape = space.addShape(new SegmentShape(staticBody, cpv(-256, 16), cpv(-192, 0), 2.0f));
		shape.setElasticity(0.0f);
		shape.setFriction(0.5f);
		shape.setFilter(NOT_GRABABLE_FILTER);

		shape = space.addShape(new SegmentShape(staticBody, cpv(-192, 0), cpv(-192, -64), 2.0f));
		shape.setElasticity(0.0f);
		shape.setFriction(0.5f);
		shape.setFilter(NOT_GRABABLE_FILTER);

		shape = space.addShape(new SegmentShape(staticBody, cpv(-128, -64), cpv(-128, 144), 2.0f));
		shape.setElasticity(0.0f);
		shape.setFriction(0.5f);
		shape.setFilter(NOT_GRABABLE_FILTER);

		shape = space.addShape(new SegmentShape(staticBody, cpv(-192, 80), cpv(-192, 176), 2.0f));
		shape.setElasticity(0.0f);
		shape.setFriction(0.5f);
		shape.setFilter(NOT_GRABABLE_FILTER);

		shape = space.addShape(new SegmentShape(staticBody, cpv(-192, 176), cpv(-128, 240), 2.0f));
		shape.setElasticity(0.0f);
		shape.setFriction(0.5f);
		shape.setFilter(NOT_GRABABLE_FILTER);

		shape = space.addShape(new SegmentShape(staticBody, cpv(-128, 144), cpv(192, 64), 2.0f));
		shape.setElasticity(0.0f);
		shape.setFriction(0.5f);
		shape.setFilter(NOT_GRABABLE_FILTER);

		Vector2f verts[] = { cpv(-30, -80), cpv(-30, 80), cpv(30, 64), cpv(30, -80), };

		Body plunger = space.addBody(new Body(1.0f, Float.POSITIVE_INFINITY));
		plunger.setPosition(cpv(-160, -80));

		shape = space.addShape(new PolyShape(plunger, 0.0f, verts, 0, 4));
		shape.setElasticity(1.0f);
		shape.setFriction(0.5f);
		shape.setFilter(new ShapeFilter(Constants.NO_GROUP, new Bitmask(1), new Bitmask(1)));

		// add balls to hopper
		for (int i = 0; i < numBalls; i++) {
			balls[i] = addBall(space, cpv(-224 + i, 80 + 64 * i));
		}

		// add small gear
		Body smallGear = space.addBody(new Body(10.0f, Util.momentForCircle(10.0f, 80, 0, cpvzero())));
		smallGear.setPosition(cpv(-160, -160));
		smallGear.setAngle(-M_PI_2);

		shape = space.addShape(new CircleShape(smallGear, 80.0f, cpvzero()));
		shape.setFilter(ShapeFilter.NONE);

		space.addConstraint(new PivotJoint(staticBody, smallGear, cpv(-160, -160), cpvzero()));

		// add big gear
		Body bigGear = space.addBody(new Body(40.0f, Util.momentForCircle(40.0f, 160, 0, cpvzero())));
		bigGear.setPosition(cpv(80, -160));
		bigGear.setAngle(M_PI_2);

		shape = space.addShape(new CircleShape(bigGear, 160.0f, cpvzero()));
		shape.setFilter(ShapeFilter.NONE);

		space.addConstraint(new PivotJoint(staticBody, bigGear, cpv(80, -160), cpvzero()));

		// connect the plunger to the small gear.
		space.addConstraint(new PinJoint(smallGear, plunger, cpv(80, 0), cpv(0, 0)));
		// connect the gears.
		space.addConstraint(new GearJoint(smallGear, bigGear, -M_PI_2, -2.0f));

		// feeder mechanism
		float bottom = -300.0f;
		float top = 32.0f;
		Body feeder = space.addBody(
				new Body(1.0f, Util.momentForSegment(1.0f, cpv(-224.0f, bottom), cpv(-224.0f, top), 0.0f)));
		feeder.setPosition(cpv(-224, (bottom + top) / 2.0f));

		float len = top - bottom;
		space.addShape(new SegmentShape(feeder, cpv(0.0f, len / 2.0f), cpv(0.0f, -len / 2.0f), 20.0f));

		space.addConstraint(new PivotJoint(staticBody, feeder, cpv(-224.0f, bottom), cpv(0.0f, -len / 2.0f)));
		Vector2f anchr = feeder.worldToLocal(cpv(-224.0f, -160.0f));
		space.addConstraint(new PinJoint(feeder, smallGear, anchr, cpv(0.0f, 80.0f)));

		// motorize the second gear
		motor = space.addConstraint(new SimpleMotor(staticBody, bigGear, 3.0f));

		return space;
	}

	@Override
	public void update(long delta) {
		float coef = (2.0f + chipmunkDemoKeyboard.getY()) / 3.0f;
		float rate = chipmunkDemoKeyboard.getX() * 30.0f * coef;

		motor.setRate(rate);
		motor.setMaxForce(rate != 0 ? 1000000.0f : 0.0f);

		int steps = 2;
		float dt = 1.0f / 60.0f / (float) steps;

		for (int j = 0; j < steps; j++) {
			space.step(dt);

			for (int i = 0; i < numBalls; i++) {
				Body ball = balls[i];
				Vector2f pos = ball.getPosition();

				if (pos.getX() > 320.0f) {
					ball.setVelocity(cpvzero());
					ball.setPosition(cpv(-224.0f, 200.0f));
				}
			}
		}
	}

	public static void main(String[] args) {
		new Pump().start(640, 480);
	}
}
