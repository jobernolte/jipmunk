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

package org.physics.jipmunk;

import java.util.List;

import static org.physics.jipmunk.Array.cpArrayDeleteObj;
import static org.physics.jipmunk.Array.cpArrayPush;
import static org.physics.jipmunk.Assert.cpAssertHard;
import static org.physics.jipmunk.Assert.cpAssertSoft;
import static org.physics.jipmunk.Body.*;
import static org.physics.jipmunk.Util.cpvlengthsq;

/** @author jobernolte */
class SpaceComponent {

	static Body ComponentRoot(Body body) {
		return (body != null ? body.sleeping.root : null);
	}

	static void ComponentActivate(Body root) {
		if (root == null || !cpBodyIsSleeping(root))
			return;
		cpAssertSoft(!cpBodyIsRogue(root), "Internal Error: ComponentActivate() called on a rogue body.");

		Space space = root.space;
		Body body = root;
		while (body != null) {
			Body next = body.sleeping.next;

			body.sleeping.idleTime = 0.0f;
			body.sleeping.root = null;
			body.sleeping.next = null;
			space.activateBody(body);

			body = next;
		}

		cpArrayDeleteObj(space.sleepingComponents, root);
	}

	static void ComponentAdd(Body root, Body body) {
		body.sleeping.root = root;

		if (body != root) {
			body.sleeping.next = root.sleeping.next;
			root.sleeping.next = body;
		}
	}

	static void cpBodyActivate(Body body) {
		body.activate();
	}

	static void cpBodyActivateStatic(Body body, Shape filter) {
		cpAssertHard(cpBodyIsStatic(body), "cpBodyActivateStatic() called on a non-static body.");

		for (Arbiter arb : body.arbiters()) {
			if (filter == null || filter == arb.a || filter == arb.b) {
				cpBodyActivate(arb.body_a == body ? arb.body_b : arb.body_a);
			}
		}

		// TODO should also activate joints?
	}

	static boolean ComponentActive(Body root, float threshold) {
		//CP_BODY_FOREACH_COMPONENT(root, body){
		for (Body body : root.components()) {
			if (body.sleeping.idleTime < threshold)
				return true;
		}

		return false;
	}

	static void FloodFillComponent(Body root, Body body) {
		// Kinematic bodies cannot be put to sleep and prevent bodies they are touching from sleeping.
		// Static bodies are effectively sleeping all the time.
		if (body.isDynamic()) {
			Body other_root = ComponentRoot(body);
			if (other_root == null) {
				ComponentAdd(root, body);
				for (Arbiter arb : body.arbiters()) {
					FloodFillComponent(root, (body == arb.body_a ? arb.body_b : arb.body_a));
				}
				for (Constraint constraint : body.constraints()) {
					FloodFillComponent(root, (body == constraint.a ? constraint.b : constraint.a));
				}
			} else {
				if (other_root != root) {
					throw new IllegalStateException("Internal Error: Inconsistency dectected in the contact graph.");
				}
			}
		}
	}

	static void cpBodyPushArbiter(Body body, Arbiter arb) {
		cpAssertSoft(arb.threadForBody(body).next == null,
					 "Internal Error: Dangling contact graph " + "pointers detected. (A)");
		cpAssertSoft(arb.threadForBody(body).prev == null,
					 "Internal Error: Dangling contact graph " + "pointers detected. (B)");

		Arbiter next = body.arbiterList;
		cpAssertSoft(next == null || next.threadForBody(body).prev == null,
					 "Internal Error: Dangling contact graph pointers detected. (C)");
		arb.threadForBody(body).next = next;

		if (next != null) {
			next.threadForBody(body).prev = arb;
		}
		body.arbiterList = arb;
	}

	static void cpSpaceProcessComponents(Space space, float dt) {
		boolean sleep = (space.sleepTimeThreshold != Float.POSITIVE_INFINITY);
		List<Body> bodies = space.dynamicBodies;

		// Calculate the kinetic energy of all the dynamicBodies.
		if (sleep) {
			float dv = space.getIdleSpeedThreshold();
			float dvsq = (dv != 0 ? dv * dv : cpvlengthsq(space.gravity) * dt * dt);

			// update idling and reset component nodes
			for (Body body : bodies) {
				// Need to deal with infinite mass objects
				float keThreshold = (dvsq != 0 ? body.m * dvsq : 0.0f);
				body.sleeping.idleTime = (body.getKineticEnergy() > keThreshold ? 0.0f : body.sleeping.idleTime + dt);

				cpAssertSoft(body.sleeping.next == null,
							 "Internal Error: Dangling next pointer detected in contact graph.");
				cpAssertSoft(body.sleeping.root == null,
							 "Internal Error: Dangling root pointer detected in contact graph.");
			}
		}

		// Awaken any sleeping dynamicBodies found and then push arbiters to the dynamicBodies' lists.
		List<Arbiter> arbiters = space.arbiters;
		// use this special iteration construct as new arbiters might be push to the arbiters list during the processing
		for (int i = 0, count = arbiters.size(); i < count; i++) {
			Arbiter arb = arbiters.get(i);
			Body a = arb.body_a;
			Body b = arb.body_b;

			if (sleep) {
				if (b.isKinematic() || a.isSleeping()) {
					a.activate();
				}
				if (a.isKinematic() || b.isSleeping()) {
					b.activate();
				}
			}

			cpBodyPushArbiter(a, arb);
			cpBodyPushArbiter(b, arb);
		}

		if (sleep) {
			// Bodies should be held active if connected by a joint to a non-static rouge body.
			List<Constraint> constraints = space.constraints;
			for (Constraint constraint : constraints) {
				Body a = constraint.a, b = constraint.b;

				if (b.isKinematic()) {
					a.activate();
				}
				if (a.isKinematic()) {
					b.activate();
				}
			}

			// Generate components and deactivate sleeping ones
			for (int i = 0; i < bodies.size(); ) {
				Body body = bodies.get(i);

				if (ComponentRoot(body) == null) {
					// Body not in a component yet. Perform a DFS to flood fill mark
					// the component in the contact graph using this body as the root.
					FloodFillComponent(body, body);

					// Check if the component should be putSingle to sleep.
					if (!ComponentActive(body, space.sleepTimeThreshold)) {
						cpArrayPush(space.sleepingComponents, body);
						//CP_BODY_FOREACH_COMPONENT(body, other)
						for (Body other : body.components()) {
							space.deactivateBody(other);
						}

						// cpSpaceDeactivateBody() removed the current body from the list.
						// Skip incrementing the index counter.
						continue;
					}
				}

				i++;

				// Only sleeping dynamicBodies retain their component sleeping pointers.
				body.sleeping.root = null;
				body.sleeping.next = null;
			}
		}
	}

	static void cpBodySleep(Body body) {
		cpBodySleepWithGroup(body, null);
	}

	static void cpBodySleepWithGroup(Body body, Body group) {
		if (!body.isDynamic()) {
			throw new IllegalStateException("Non-dynamic bodies cannot be put to sleep.");
		}

		Space space = body.space;
		if (space.locked != 0) {
			throw new IllegalStateException(
					"Bodies cannot be put to sleep during a query or a call to cpSpaceStep(). Put these calls into a post-step callback.");
		}
		if (group != null && !group.isSleeping()) {
			throw new IllegalStateException("Cannot use a non-sleeping body as a group identifier.");
		}

		if (body.isSleeping() && ComponentRoot(body) != ComponentRoot(group)) {
			throw new IllegalStateException("The body is already sleeping and it's group cannot be reassigned.");
		}

		for (Shape shape : body.shapes()) {
			shape.cacheBB();
		}
		space.deactivateBody(body);

		if (group != null) {
			Body root = ComponentRoot(group);

			body.sleeping.root = root;
			body.sleeping.next = root.sleeping.next;
			body.sleeping.idleTime = 0.0f;

			root.sleeping.next = body;
		} else {
			body.sleeping.root = body;
			body.sleeping.next = null;
			body.sleeping.idleTime = 0.0f;

			cpArrayPush(space.sleepingComponents, body);
		}

		cpArrayDeleteObj(space.dynamicBodies, body);
	}

}
