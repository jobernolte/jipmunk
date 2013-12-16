/*
 * Copyright (c) 2007 Scott Lembcke, (c) 2011 Jürgen Obernolte 
 * this example only (c) 2011 Chris Camacho
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.physics.jipmunk.examples;

import static org.physics.jipmunk.Util.cpv;

import java.util.Random;

import org.physics.jipmunk.*;

/** @author chris_c based on work by jobernolte & Lembcke */
public class Plink extends ExampleBase {

	private static Random rgen = new Random();
	private bodyCheckIterator checkBodies = new bodyCheckIterator();
	private Space space;
	private static final float pentagon_mass = 0.0f;
	private static final float pentagon_moment = 0.0f;

	// rather than anon inner class...
	class bodyCheckIterator implements SpaceBodyIteratorFunc {

		// Iterate over all of the bodies and reset the ones that have fallen offscreen.
		public void visit(Body body) {
			Vector2f pos = body.getPosition();
			if (pos.getY() < -260 || Util.cpfabs(pos.getY()) > 340) {
				float x = rgen.nextFloat() * 640 - 320;
				body.setPosition(cpv(x, 260));
			}
		}
	}

	@Override
	public Space init() {
		System.out.println("initializing plink");
		space = new Space();

		space.setIterations(5);
		space.setGravity(Util.cpv(0, -100));
		space.setSleepTimeThreshold(0.5f);
		space.setCollisionSlop(0.5f);

		Body body, staticBody = space.getStaticBody();
		Shape shape;

		// Vertexes for a triangle shape.
		Vector2f tris[] = { Util.cpv(-15f, -15f), Util.cpv(0f, 10f), Util.cpv(15f, -15f) };

		// Create the static triangles.
		for (int i = 0; i < 9; i++) {
			for (int j = 0; j < 6; j++) {
				float stagger = (j % 2) * 40f;
				Vector2f offset = cpv(i * 80f - 320f + stagger, j * 70f - 240f);
				shape = space.addShape(new PolyShape(staticBody, 0.0f, Transform.translate(offset), tris));
				shape.setElasticity(1.0f);
				shape.setFriction(0.1f);
				shape.setFilter(NOT_GRABABLE_FILTER);
			}
		}

		// Create vertexes for a pentagon shape.
		final int NUM_VERTS = 5;
		Vector2f[] verts = new Vector2f[NUM_VERTS];
		for (int i = 0; i < NUM_VERTS; i++) {
			float angle = -2f * (float) Math.PI * i / ((float) NUM_VERTS);
			verts[i] = Util.cpv(10f * (float) Math.cos(angle), 10 * (float) Math.sin(angle));
		}

		// Add lots of pentagons.
		for (int i = 0; i < 300; i++) {
			body = space.addBody(new Body(1.0f, Util.momentForPoly(1.0f, verts, Util.cpvzero(), 0.0f)));
			float x = rgen.nextFloat() * 640f - 320f;
			body.setPosition(Util.cpv(x, 350f));

			shape = space.addShape(new PolyShape(body, 0.0f, verts));
			shape.setElasticity(0.1f);
			shape.setFriction(0.4f);
		}

		return space;
	}

	@Override
	public void update(long delta) {
		/*
		 * int steps = 1; float dt = 1.0f / 60.0f / (float) steps;
		 * 
		 * for (int i = 0; i < steps; i++) { space.step(dt); space.eachBody(checkBodies); }
		 */
		if (chipmunkDemoRightDown) {
			Shape nearest = space.pointQueryNearest(mousePoint, 0.0f, GRAB_FILTER, null).shape;
			if (nearest != null) {
				Body body = nearest.getBody();
				if (body.isStatic()) {
					space.convertBodyToDynamic(body, pentagon_mass, pentagon_moment);
					space.addBody(body);
				} else {
					space.removeBody(body);
					space.convertBodyToStatic(body);
				}
			}
		}

		int steps = 1;
		float dt = 1.0f / 60.0f / (float) steps;

		for (int i = 0; i < steps; i++) {
			space.step(dt);
			space.eachBody(checkBodies);
		}
	}

	public static void main(String[] args) {
		new Plink().start(640, 480);
	}
}
