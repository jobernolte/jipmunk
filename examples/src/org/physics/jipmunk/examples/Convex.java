package org.physics.jipmunk.examples;

import org.physics.jipmunk.Body;
import org.physics.jipmunk.ConvexHullInfo;
import org.physics.jipmunk.ConvexHullUtil;
import org.physics.jipmunk.DefaultVector2f;
import org.physics.jipmunk.PolyShape;
import org.physics.jipmunk.SegmentShape;
import org.physics.jipmunk.Shape;
import org.physics.jipmunk.Space;
import org.physics.jipmunk.Util;
import org.physics.jipmunk.Vector2f;
import org.physics.jipmunk.constraints.NearestPointQueryInfo;

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
		shape.setFrictionCoefficient(1.0f);
		shape.setLayers(NOT_GRABABLE_MASK);

		float width = 50.0f;
		float height = 70.0f;
		float mass = width * height * DENSITY;
		float moment = Util.momentForBox(mass, width, height);

		body = space.addBody(new Body(mass, moment));

		shape = space.addShape(PolyShape.createBox(body, width, height));
		shape.setFrictionCoefficient(0.6f);
		this.shape = (PolyShape) shape;
		return space;
	}

	@Override
	public void update(long delta) {
		float tolerance = 2.0f;

		NearestPointQueryInfo info = shape.nearestPointQuery(mousePoint, null);
		if (chipmunkDemoRightClick && info.d > tolerance) {
			Body body = shape.getBody();
			int count = shape.getNumVertices();

			// Allocate the space for the new vertexes on the stack.
			//cpVect * verts = (cpVect *) alloca((count + 1) * sizeof(cpVect));
			Vector2f[] verts = new DefaultVector2f[count + 1];

			for (int i = 0; i < count; i++) {
				verts[i] = new DefaultVector2f(shape.getVertices()[i]);
			}

			verts[count] = body.world2Local(mousePoint); //  cpBodyWorld2Local(body, ChipmunkDemoMouse);

			// System.out.println("adding point " + verts[count]);

			// This function builds a convex hull for the vertexes.
			// Because the result array is NULL, it will reduce the input array instead.
			ConvexHullInfo convexHullInfo = ConvexHullUtil.convexHull(count + 1, verts, null, tolerance);
			int hullCount = convexHullInfo.count;

			/*for (int i = 0; i < hullCount; i++) {
				System.out.format("%d: [%f,%f]\n", i, verts[i].getX(), verts[i].getY());
			}*/

			// Figure out how much to shift the body by.
			Vector2f centroid = Util.centroidForPoly(verts, 0, hullCount);

			// Recalculate the body properties to match the updated shape.
			float mass = Util.areaForPoly(verts, 0, hullCount) * DENSITY;
			body.setMass(mass);
			body.setMoment(Util.momentForPoly(mass, verts, 0, hullCount, Util.cpvneg(centroid)));
			body.setPosition(body.local2World(centroid));

			// Use the setter function from chipmunk_unsafe.h.
			// You could also remove and recreate the shape if you wanted.
			shape.setVertices(verts, 0, hullCount, Util.cpvneg(centroid));
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
