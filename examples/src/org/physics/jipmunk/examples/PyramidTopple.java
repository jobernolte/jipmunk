package org.physics.jipmunk.examples;

import org.physics.jipmunk.*;

import static org.physics.jipmunk.Util.cpv;

/**
 * @author chris_c based on Chipmunk Demo
 */
public class PyramidTopple extends ExampleBase {
	private Space space;
	private static final float WIDTH = 4.0f;
	private final static float HEIGHT = 30.0f;

	private void add_domino(Vector2f pos, boolean flipped) {
		float mass = 1.0f;
		float moment = Util.momentForBox(mass, WIDTH, HEIGHT);

		Body body = space.addBody(new Body(mass, moment));
		body.setPosition(pos);

		Shape shape = (flipped ? PolyShape.createBox(body, HEIGHT, WIDTH, 0.0f) :
				PolyShape.createBox(body, WIDTH, HEIGHT, 0.0f));
		space.addShape(shape);
		shape.setElasticity(0.0f);
		shape.setFriction(0.6f);
	}

	@Override
	public Space init() {
		System.out.println("initializing pyramid topple");
		space = new Space();

		space.setIterations(30);
		space.setGravity(Util.cpv(0, -100));
		space.setSleepTimeThreshold(0.5f);
		space.setCollisionSlop(0.5f);

		Body body, staticBody = space.getStaticBody();
		Shape shape;

		float hw = getWidth() / 2.0f;
		float hh = getHeight() / 2.0f;

		// Create floor

		shape = space.addShape(new SegmentShape(staticBody, cpv(-hw, -hh), cpv(hw, -hh), 0.0f));
		shape.setElasticity(1.0f);
		shape.setFriction(1.0f);
		shape.setFilter(NOT_GRABABLE_FILTER);

		// Add the dominoes.
		int n = 12;
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < (n - i); j++) {
				Vector2f offset =
						cpv((j - (n - 1 - i) * 0.5f) * 1.5f * HEIGHT, (i + 0.5f) * (HEIGHT + 2 * WIDTH) - WIDTH - 230);
				add_domino(offset, false);
				add_domino(Util.cpvadd(offset, cpv(0, (HEIGHT + WIDTH) / 2.0f)), true);

				if (j == 0) {
					add_domino(Util.cpvadd(offset, cpv(0.5f * (WIDTH - HEIGHT), HEIGHT + WIDTH)), false);
				}

				if (j != n - i - 1) {
					add_domino(Util.cpvadd(offset, cpv(HEIGHT * 0.75f, (HEIGHT + 3 * WIDTH) / 2.0f)), true);
				} else {
					add_domino(Util.cpvadd(offset, cpv(0.5f * (HEIGHT - WIDTH), HEIGHT + WIDTH)), false);
				}
			}
		}

		return space;
	}

	@Override
	public void update(long delta) {
		int steps = 3;
		float dt = 1.0f / 60.0f / (float) steps;

		for (int i = 0; i < steps; i++) {
			space.step(dt);
		}
	}

	public static void main(String[] args) {
		new PyramidTopple().start(640, 480);
	}
}