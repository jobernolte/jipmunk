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
import static org.physics.jipmunk.Body.cpBodyIsRogue;
import static org.physics.jipmunk.Body.cpBodyIsSleeping;
import static org.physics.jipmunk.Body.cpBodyIsStatic;
import static org.physics.jipmunk.Body.cpBodyKineticEnergy;
import static org.physics.jipmunk.Shape.cpShapeUpdate;
import static org.physics.jipmunk.Space.cpSpaceUncacheArbiter;
import static org.physics.jipmunk.SpaceQuery.cpSpaceShapeQuery;
import static org.physics.jipmunk.SpatialIndex.cpSpatialIndexInsert;
import static org.physics.jipmunk.SpatialIndex.cpSpatialIndexRemove;
import static org.physics.jipmunk.Util.cpvlengthsq;

/** @author jobernolte */
class SpaceComponent {

	static void cpSpaceActivateBody(Space space, Body body) {
		space.activateBody(body);
	}

	static void cpSpaceDeactivateBody(Space space, Body body) {
		cpAssertSoft(!cpBodyIsRogue(body), "Internal error: Attempting to deactivate a rouge body.");

		cpArrayDeleteObj(space.bodies, body);

		//CP_BODY_FOREACH_SHAPE(body, shape){
		for (Shape shape : body.shapes()) {
			cpSpatialIndexRemove(space.activeShapes, shape, shape.hashid);
			cpSpatialIndexInsert(space.staticShapes, shape, shape.hashid);
		}

		//CP_BODY_FOREACH_ARBITER(body, arb){
		for (Arbiter arb : body.arbiters()) {
			Body bodyA = arb.body_a;
			if (body == bodyA || cpBodyIsStatic(bodyA)) {
				cpSpaceUncacheArbiter(space, arb);

				// Save contact data to a new block of memory so they won't time out
				/*TODO size_t bytes = arb.numContacts*sizeof(cpContact);
								cpContact *contacts = (cpContact *)cpcalloc(1, bytes);
								memcpy(contacts, arb.contacts, bytes);
								arb.contacts = contacts;*/
			}
		}

		for (Constraint constraint : body.constraints()) {
			Body bodyA = constraint.a;
			if (body == bodyA || cpBodyIsStatic(bodyA)) cpArrayDeleteObj(space.constraints, constraint);
		}
	}

	static Body ComponentRoot(Body body) {
		return (body != null ? body.node.root : null);
	}

	static void ComponentActivate(Body root) {
		if (root == null || !cpBodyIsSleeping(root)) return;
		cpAssertSoft(!cpBodyIsRogue(root), "Internal Error: ComponentActivate() called on a rogue body.");

		Space space = root.space;
		Body body = root;
		while (body != null) {
			Body next = body.node.next;

			body.node.idleTime = 0.0f;
			body.node.root = null;
			body.node.next = null;
			cpSpaceActivateBody(space, body);

			body = next;
		}

		cpArrayDeleteObj(space.sleepingComponents, root);
	}

	static void ComponentAdd(Body root, Body body) {
		body.node.root = root;

		if (body != root) {
			body.node.next = root.node.next;
			root.node.next = body;
		}
	}

	static void cpBodyActivate(Body body) {
		/*if (!cpBodyIsRogue(body)) {
					body.node.idleTime = 0.0f;
					ComponentActivate(ComponentRoot(body));
				}*/
		body.activate();
	}

	static void cpBodyActivateStatic(Body body, Shape filter) {
		cpAssertHard(cpBodyIsStatic(body), "cpBodyActivateStatic() called on a non-static body.");

		//CP_BODY_FOREACH_ARBITER(body, arb){
		for (Arbiter arb : body.arbiters()) {
			if (filter == null || filter == arb.a || filter == arb.b) {
				cpBodyActivate(arb.body_a == body ? arb.body_b : arb.body_a);
			}
		}
	}

	static boolean ComponentActive(Body root, float threshold) {
		//CP_BODY_FOREACH_COMPONENT(root, body){
		for (Body body : root.components()) {
			if (body.node.idleTime < threshold) return true;
		}

		return false;
	}

	static void FloodFillComponent(Body root, Body body) {
		// Rogue bodies cannot be putSingle to sleep and prevent bodies they are touching from sleepining anyway.
		// Static bodies (which are a type of rogue body) are effectively sleeping all the time.
		if (!cpBodyIsRogue(body)) {
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
				cpAssertSoft(other_root == root, "Internal Error: Inconsistency dectected in the contact graph.");
			}
		}
	}

	static void cpBodyPushArbiter(Body body, Arbiter arb) {
		cpAssertSoft(arb.threadForBody(body).next == null, "Internal Error: Dangling contact graph " +
				"pointers detected. (A)");
		cpAssertSoft(arb.threadForBody(body).prev == null, "Internal Error: Dangling contact graph " +
				"pointers detected. (B)");

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
		float dv = space.getIdleSpeedThreshold();
		float dvsq = (dv != 0 ? dv * dv : cpvlengthsq(space.gravity) * dt * dt);

		// update idling and reset component nodes
		List<Body> bodies = space.bodies;
		for (int i = 0; i < bodies.size(); i++) {
			Body body = bodies.get(i);

			// Need to deal with infinite mass objects
			float keThreshold = (dvsq != 0 ? body.m * dvsq : 0.0f);
			body.node.idleTime = (cpBodyKineticEnergy(body) > keThreshold ? 0.0f : body.node.idleTime + dt);

			cpAssertSoft(body.node.next == null, "Internal Error: Dangling next pointer detected in contact graph.");
			cpAssertSoft(body.node.root == null, "Internal Error: Dangling root pointer detected in contact graph.");
		}

		// Awaken any sleeping bodies found and then push arbiters to the bodies' lists.
		List<Arbiter> arbiters = space.arbiters;
		for (int i = 0, count = arbiters.size(); i < count; i++) {
			Arbiter arb = arbiters.get(i);
			Body a = arb.body_a;
			Body b = arb.body_b;

			if ((cpBodyIsRogue(b) && !cpBodyIsStatic(b)) || cpBodyIsSleeping(a)) cpBodyActivate(a);
			if ((cpBodyIsRogue(a) && !cpBodyIsStatic(a)) || cpBodyIsSleeping(b)) cpBodyActivate(b);

			cpBodyPushArbiter(a, arb);
			cpBodyPushArbiter(b, arb);
		}

		// Bodies should be held active if connected by a joint to a non-static rouge body.
		List<Constraint> constraints = space.constraints;
		for (int i = 0; i < constraints.size(); i++) {
			Constraint constraint = constraints.get(i);
			Body a = constraint.a, b = constraint.b;

			if (cpBodyIsRogue(b) && !cpBodyIsStatic(b)) cpBodyActivate(a);
			if (cpBodyIsRogue(a) && !cpBodyIsStatic(a)) cpBodyActivate(b);
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
						cpSpaceDeactivateBody(space, other);
					}

					// cpSpaceDeactivateBody() removed the current body from the list.
					// Skip incrementing the index counter.
					continue;
				}
			}

			i++;

			// Only sleeping bodies retain their component node pointers.
			body.node.root = null;
			body.node.next = null;
		}
	}

	static void cpBodySleep(Body body) {
		cpBodySleepWithGroup(body, null);
	}

	static void cpBodySleepWithGroup(Body body, Body group) {
		cpAssertHard(!cpBodyIsStatic(body) && !cpBodyIsRogue(body),
				"Rogue and static bodies cannot be putSingle to sleep.");

		Space space = body.space;
		cpAssertHard(space != null, "Cannot putSingle a rogue body to sleep.");
		cpAssertHard(space.locked == 0,
				"Bodies cannot be putSingle to sleep during a query or a call to cpSpaceStep(). " +
						"Put" +
						" " +
						"these calls into a post-step callback.");
		cpAssertHard(group == null || cpBodyIsSleeping(group), "Cannot use a non-sleeping body as a group identifier" +
				".");

		if (cpBodyIsSleeping(body)) {
			cpAssertHard(ComponentRoot(body) == ComponentRoot(group), "The body is already sleeping and it's group " +
					"cannot be reassigned.");
			return;
		}

		//CP_BODY_FOREACH_SHAPE(body, shape)
		for (Shape shape : body.shapes()) {
			cpShapeUpdate(shape, body.p, body.rot);
		}
		cpSpaceDeactivateBody(space, body);

		if (group != null) {
			Body root = ComponentRoot(group);

			body.node = new Body.ComponentNode(root, root.node.next, 0.0f);

			root.node.next = body;
		} else {
			body.node = new Body.ComponentNode(body, null, 0.0f);

			cpArrayPush(space.sleepingComponents, body);
		}

		cpArrayDeleteObj(space.bodies, body);
	}

	/*static void
		activateTouchingHelper(Shape shape, ContactPointSet points, Shape other) {
			cpBodyActivate(shape.body);
		}*/

	static void cpSpaceActivateShapesTouchingShape(Space space, Shape shape) {
		if (space.sleepTimeThreshold != Float.POSITIVE_INFINITY) {
			cpSpaceShapeQuery(space, shape, new SpaceShapeQueryFunc() {
				@Override
				public void apply(Shape shape, ContactPointSet contactPointSet) {
					cpBodyActivate(shape.body);
				}
			});
		}
	}

}
