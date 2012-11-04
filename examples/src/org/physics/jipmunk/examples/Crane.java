package org.physics.jipmunk.examples;

import org.physics.jipmunk.Arbiter;
import org.physics.jipmunk.Body;
import org.physics.jipmunk.CircleShape;
import org.physics.jipmunk.Constraint;
import org.physics.jipmunk.DefaultCollisionHandler;
import org.physics.jipmunk.PolyShape;
import org.physics.jipmunk.PostStepFunc;
import org.physics.jipmunk.SegmentShape;
import org.physics.jipmunk.Shape;
import org.physics.jipmunk.Space;
import org.physics.jipmunk.Util;
import org.physics.jipmunk.constraints.GrooveJoint;
import org.physics.jipmunk.constraints.PivotJoint;
import org.physics.jipmunk.constraints.SlideJoint;

import static org.physics.jipmunk.Util.cpfmax;
import static org.physics.jipmunk.Util.cpv;
import static org.physics.jipmunk.Util.cpvzero;

/** @author jobernolte */
public class Crane extends ExampleBase {
	private Space space;
	private Body dollyBody;
	// Constraint used as a servo motor to move the dolly back and forth.
	private PivotJoint dollyServo;

	// Constraint used as a winch motor to lift the load.
	private SlideJoint winchServo;

	// Temporary joint used to hold the hook to the load.
	private Constraint hookJoint;

	private final static int HOOK_SENSOR = 1;
	private final static int CRATE = 2;

	private void attachHook(Space space, Body hook, Body crate) {
		hookJoint = space.addConstraint(new PivotJoint(hook, crate, hook.getPosition()));
	}

	private boolean hookCrate(Arbiter arb, Space space) {
		if (hookJoint == null) {
			// Get pointers to the two bodies in the collision pair and define local variables for them.
			// Their order matches the order of the collision types passed
			// to the collision handler this function was defined for
			final Body hook = arb.getBodyA();
			final Body crate = arb.getBodyB();

			// additions and removals can't be done in a normal callback.
			// Schedule a post step callback to do it.
			// Use the hook as the key and pass along the arbiter.
			space.addPostStepCallback(new PostStepFunc() {
				@Override
				public void call(Space space) {
					attachHook(space, hook, crate);
				}
			});
		}

		return true; // return value is ignored for sensor callbacks anyway
	}

	@Override
	public Space init() {
		this.space = new Space();
		space.setIterations(30);
		space.setGravity(cpv(0, -100));
		space.setDamping(0.8f);

		Body staticBody = space.getStaticBody();
		Shape shape;

		shape = space.addShape(new SegmentShape(staticBody, cpv(-320, -240), cpv(320, -240), 0.0f));
		shape.setElasticity(1.0f);
		shape.setFrictionCoefficient(1.0f);
		shape.setLayers(NOT_GRABABLE_MASK);

		// Add a body for the dolly.
		this.dollyBody = space.addBody(new Body(10, Float.POSITIVE_INFINITY));
		dollyBody.setPosition(cpv(0, 100));

		// Add a block so you can see it.
		space.addShape(PolyShape.createBox(dollyBody, 30, 30));

		// Add a groove joint for it to move back and forth on.
		space.addConstraint(new GrooveJoint(staticBody, dollyBody, cpv(-250, 100), cpv(250, 100), cpvzero()));

		// Add a pivot joint to act as a servo motor controlling it's position
		// By updating the anchor points of the pivot joint, you can move the dolly.
		this.dollyServo = space.addConstraint(new PivotJoint(staticBody, dollyBody, dollyBody.getPosition()));
		// Max force the dolly servo can generate.
		dollyServo.setMaxForce(10000);
		// Max speed of the dolly servo
		dollyServo.setMaxBias(100);
		// You can also change the error bias to control how it slows down.
		//cpConstraintSetErrorBias(dollyServo, 0.2);

		// Add the crane hook.
		Body hookBody = space.addBody(new Body(1, Float.POSITIVE_INFINITY));
		hookBody.setPosition(cpv(0, 50));

		// Add a sensor shape for it. This will be used to figure out when the hook touches a box.
		shape = space.addShape(new CircleShape(hookBody, 10, cpvzero()));
		shape.setSensor(true);
		shape.setCollisionType(HOOK_SENSOR);

		// Add a slide joint to act as a winch motor
		// By updating the max length of the joint you can make it pull up the load.
		this.winchServo = space.addConstraint(new SlideJoint(dollyBody, hookBody, cpvzero(), cpvzero(), 0,
				Float.POSITIVE_INFINITY));
		// Max force the dolly servo can generate.
		winchServo.setMaxForce(30000);
		// Max speed of the dolly servo
		winchServo.setMaxBias(60);

		// TODO cleanup
		// Finally a box to play with
		Body boxBody = space.addBody(new Body(30, Util.momentForBox(30, 50, 50)));
		boxBody.setPosition(cpv(200, -200));

		// Add a block so you can see it.
		shape = space.addShape(PolyShape.createBox(boxBody, 50, 50));
		shape.setFrictionCoefficient(0.7f);
		shape.setCollisionType(CRATE);

		space.addCollisionHandler(HOOK_SENSOR, CRATE, new DefaultCollisionHandler() {
			@Override
			public boolean begin(Arbiter arb, Space space) {
				return hookCrate(arb, space);
			}

		});

		return space;
	}

	@Override
	public void update(long delta) {
		int steps = 1;
		float dt = 1.0f / 60.0f / (float) steps;

		for (int i = 0; i < steps; i++) {
			// Set the first anchor point (the one attached to the static body) of the dolly servo to the mouse's x position.
			dollyServo.setAnchr1(cpv(mousePoint.getX(), 100));

			// Set the max length of the winch servo to match the mouse's height.
			winchServo.setMax(cpfmax(100 - mousePoint.getY(), 50));

			if (hookJoint != null && chipmunkDemoRightClick) {
				space.removeConstraint(hookJoint);
				hookJoint = null;
			}

			space.step(dt);
		}
	}

	public static void main(String[] args) {
		new Crane().start(640, 480);
	}
}
