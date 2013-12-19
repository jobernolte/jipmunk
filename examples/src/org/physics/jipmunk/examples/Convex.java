package org.physics.jipmunk.examples;

import org.physics.jipmunk.*;
import org.physics.jipmunk.constraints.PointQueryInfo;

/** @author jobernolte */
public class Convex extends ExampleBase {

	private final static float DENSITY = 1.0f / 10000.0f;
	private Space space;
	private PolyShape shape;

	@Override
	public Space init() {
		this.space = new Space();
		space.setIterations(30);
		space.setGravity(Util.cpv(0, -500));
		space.setSleepTimeThreshold(0.5f);
		space.setCollisionSlop(0.5f);

		Body body, staticBody = space.getStaticBody();

		// Create segments around the edge of the screen.
		float hw = getWidth() / 2.0f;
		float hh = getHeight() / 2.0f;
		Shape shape = space.addShape(new SegmentShape(staticBody, Util.cpv(-hw, -hh), Util.cpv(hw, -hh), 0.0f));
		shape.setElasticity(1.0f);
		shape.setFriction(1.0f);
		shape.setFilter(NOT_GRABABLE_FILTER);

		float width = 50.0f;
		float height = 70.0f;
		float mass = width * height * DENSITY;
		float moment = Util.momentForBox(mass, width, height);

		body = space.addBody(new Body(mass, moment));

		shape = space.addShape(PolyShape.createBox(body, width, height, 0.0f));
		shape.setFriction(0.6f);
		this.shape = (PolyShape) shape;
		return space;
	}

	@Override
	public void update(long delta) {
		float tolerance = 2.0f;

		PointQueryInfo info = shape.pointQuery(mousePoint, null);
		if (chipmunkDemoRightClick && info.distance > tolerance) {
			Body body = shape.getBody();
			int count = shape.getNumVertices();

			// Allocate the space for the new vertexes on the stack.
			//cpVect * verts = (cpVect *) alloca((count + 1) * sizeof(cpVect));
			Vector2f[] verts = new Vector2f[count + 1];

			for (int i = 0; i < count; i++) {
				verts[i] = new Vector2f(shape.getVertexAt(i));
			}

			verts[count] = body.worldToLocal(mousePoint); //  cpBodyWorld2Local(body, ChipmunkDemoMouse);

			// System.out.println("adding point " + verts[count]);

			// This function builds a convex hull for the vertexes.
			// Because the result array is NULL, it will reduce the input array instead.
			ConvexHullInfo convexHullInfo = ConvexHullUtil.convexHull(verts, null, count + 1, tolerance);
			int hullCount = convexHullInfo.count;

			/*for (int i = 0; i < hullCount; i++) {
				System.out.format("%distance: [%f,%f]\normal", i, verts[i].getX(), verts[i].getY());
			}*/

			// Figure out how much to shift the body by.
			Vector2f centroid = Util.centroidForPoly(verts, 0, hullCount);

			// Recalculate the body properties to match the updated shape.
			float mass = Util.areaForPoly(verts, 0, hullCount, 0.0f) * DENSITY;
			body.setMass(mass);
			body.setMoment(Util.momentForPoly(mass, verts, 0, hullCount, Util.cpvneg(centroid), 0.0f));
			body.setPosition(body.localToWorld(centroid));

			// Use the setter function from chipmunk_unsafe.h.
			// You could also remove and recreate the shape if you wanted.
			shape.setVertices(verts, 0, hullCount, Transform.translate(Util.cpvneg(centroid)));
		}

		int steps = 1;
		float dt = 1.0f / 60.0f / (float) steps;

		for (int i = 0; i < steps; i++) {
			space.step(dt);
		}
	}

	public static void main(String[] args) {
		new Convex().start(640, 480);
	}
}
