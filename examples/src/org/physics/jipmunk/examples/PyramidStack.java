/*
 * Copyright (c) 2007 Scott Lembcke, (c) 2011 Jürgen Obernolte
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

import org.physics.jipmunk.Body;
import org.physics.jipmunk.CircleShape;
import org.physics.jipmunk.PolyShape;
import org.physics.jipmunk.SegmentShape;
import org.physics.jipmunk.Shape;
import org.physics.jipmunk.Space;
import org.physics.jipmunk.Util;

import static org.physics.jipmunk.Util.cpv;

/** @author jobernolte */
public class PyramidStack extends ExampleBase {
	private Space space;

	@Override
	public Space init() {
		System.out.println("initializing pyramid stack");
		space = new Space();

		space.setIterations(30);
		space.setGravity(Util.cpv(0, -100));
		space.setSleepTimeThreshold(0.5f);
		space.setCollisionSlop(0.5f);

		Body body, staticBody = space.getStaticBody();
		Shape shape;

		float hw = getWidth() / 2.0f;
		float hh = getHeight() / 2.0f;

		// Create segments around the edge of the screen.

		shape = space.addShape(new SegmentShape(staticBody, cpv(-hw, -hh), cpv(-hw, hh), 0.0f));
		shape.setElasticity(1.0f);
		shape.setFrictionCoefficient(1.0f);
		shape.setLayers(NOT_GRABABLE_MASK);

		shape = space.addShape(new SegmentShape(staticBody, cpv(hw, -hh), cpv(hw, hh), 0.0f));
		shape.setElasticity(1.0f);
		shape.setFrictionCoefficient(1.0f);
		shape.setLayers(NOT_GRABABLE_MASK);

		shape = space.addShape(new SegmentShape(staticBody, cpv(-hw, -hh), cpv(hw, -hh), 0.0f));
		shape.setElasticity(1.0f);
		shape.setFrictionCoefficient(1.0f);
		shape.setLayers(NOT_GRABABLE_MASK);

		// Add lots of boxes.
		for (int i = 0; i < 14; i++) {
			for (int j = 0; j <= i; j++) {
				body = space.addBody(new Body(1.0f, Util.momentForBox(1.0f, 30.0f, 30.0f)));
				body.setPosition(cpv(j * 32 - i * 16, 300 - i * 32));

				shape = space.addShape(PolyShape.createBox(body, 30.0f, 30.0f));
				shape.setElasticity(0.0f);
				shape.setFrictionCoefficient(0.8f);
			}
		}

		// Add a ball to make things more interesting
		float radius = 15.0f;
		body = space.addBody(new Body(10.0f, Util.momentForCircle(10.0f, 0.0f, radius, Util.cpvzero())));
		body.setPosition(cpv(0, -hh + radius + 5));

		shape = space.addShape(new CircleShape(body, radius, Util.cpvzero()));
		shape.setElasticity(0.0f);
		shape.setFrictionCoefficient(0.9f);

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
		new PyramidStack().start(640, 480);
	}
}
