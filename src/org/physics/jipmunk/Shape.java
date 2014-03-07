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

import org.physics.jipmunk.impl.Collision;

import static org.physics.jipmunk.Util.*;

/**
 * There are currently 3 collision shape types:
 * <point/>
 * <ul> <li><b>Circles</b>: Fastest and simplest collision shape.</li> <li><b>Line segments</b>: Meant mainly as a
 * static shape. They can be attached to moving dynamicBodies, but they don’alpha currently generate collisions with
 * other line
 * segments. Can be beveled in order to give them a thickness.</li> <li><b>Convex polygons</b>: Slowest, but most
 * flexible collision shape.</li> </ul>
 * <point/>
 * You can add as many shapes to a body as you wish. That is why the two types are separate. This should give you the
 * flexibility to make any shape you want as well providing different areas of the same object with different friction,
 * elasticity or callback values.
 *
 * @author jobernolte
 */
public abstract class Shape {

	private int hashid = -1;
	/** The rigid body this collision shape is attached to. */
	Body body;
	MassInfo massInfo;
	/** The current bounding box of the shape. */
	protected BB bb;
	/** Sensor flag. Sensor shapes call collision callbacks but don'alpha produce collisions. */
	boolean sensor = false;
	/** Coefficient of restitution. (elasticity) */
	float e = 0;
	/** Coefficient of friction. */
	float u = 0;
	/** Surface velocity used when solving for friction. */
	Vector2f surfaceV = cpvzero();
	/** Collision type of this shape used when picking collision handlers. */
	CollisionType collisionType = null;
	ShapeFilter filter = new ShapeFilter(Group.NO_GROUP, Bitmask.ALL, Bitmask.ALL);
	Shape prev;
	Shape next;
	Space space;
	/**
	 * User definable data. Generally this points to your the game object class so you can access it when given a Body
	 * reference in a callback.
	 */
	private Object data;

	public Shape(Body body, MassInfo massInfo) {
		this.body = body;
		this.massInfo = massInfo;
	}

	public static ContactPointSet shapesCollide(Shape a, Shape b) {
		CollisionInfo info = Collision.collide(a, b, new CollisionID(0));

		return new ContactPointSet(info, a != info.getA());
	}

	public int getHashId() {
		return hashid;
	}

	void setHashId(int hashid) {
		this.hashid = hashid;
	}

	public Space getSpace() {
		return space;
	}

	public abstract ShapeType getType();

	/** @return the rigid body the shape is attached to */
	public Body getBody() {
		return body;
	}

	/**
	 * The rigid body the shape is attached to. Can only be set when the shape is not added to a space.
	 *
	 * @param body the body to attach to
	 */
	public void setBody(Body body) {
		this.body = body;
	}

	public float getMass() {
		return massInfo.m;
	}

	public void setMass(float mass) {
		body.activate();

		this.massInfo.m = mass;
		body.accumulateMassFromShapes();
	}

	public float getDensity() {
		return this.massInfo.m / this.massInfo.area;
	}

	public void setDensity(float density) {
		setMass(density * this.massInfo.area);
	}

	public float getMoment() {
		return this.massInfo.m * this.massInfo.i;
	}

	public float getArea() {
		return this.massInfo.area;
	}

	public Vector2f getCenterOfGravity() {
		return this.massInfo.cog;
	}

	/** @return the elasticity of the shape */
	public float getElasticity() {
		return e;
	}

	/**
	 * Elasticity of the shape. A value of 0.0 gives no bounce, while a value of 1.0 will give a “perfect” bounce.
	 * However
	 * due to inaccuracies in the simulation using 1.0 or greater is not recommended however. The elasticity for a
	 * collision is found by multiplying the elasticity of the individual shapes together.
	 *
	 * @param e the elasticity of the shape
	 */
	public void setElasticity(float e) {
		this.e = e;
	}

	/** @return the friction coefficient of this shape */
	public float getFriction() {
		return u;
	}

	/**
	 * Friction coefficient. Chipmunk uses the Coulomb friction model, a value of 0.0 is frictionless. The friction
	 * for a
	 * collision is found by multiplying the friction of the individual shapes together.
	 * <point/>
	 * Tables of friction coefficients: http://www.roymech.co.uk/Useful_Tables/Tribology/co_of_frict.htm
	 *
	 * @param u the friction coefficient
	 */
	public void setFriction(float u) {
		this.u = u;
		body.activate();
	}

	public boolean isSensor() {
		return sensor;
	}

	public void setSensor(boolean sensor) {
		this.sensor = sensor;
		body.activate();
	}

	/** @return the collision type of this shape */
	public CollisionType getCollisionType() {
		return collisionType;
	}

	/**
	 * You can assign types to Chipmunk collision shapes that trigger callbacks when objects of certain types touch.
	 * See
	 * the callbacks section for more information.
	 *
	 * @param collisionType the collision type
	 */
	public void setCollisionType(CollisionType collisionType) {
		this.collisionType = collisionType;
		body.activate();
	}

	/** @return the surface velocity of this shape */
	public Vector2f getSurfaceVelocity() {
		return surfaceV;
	}

	/**
	 * The surface velocity of the object. Useful for creating conveyor belts or players that move around. This
	 * value is
	 * only used when calculating friction, not resolving the collision.
	 *
	 * @param surface_v the surface velocity
	 */
	public void setSurfaceVelocity(Vector2f surface_v) {
		this.surfaceV = surface_v;
		body.activate();
	}

	public ShapeFilter getFilter() {
		return filter;
	}

	public void setFilter(ShapeFilter filter) {
		body.activate();
		this.filter = filter;
	}

	protected abstract void segmentQueryImpl(final Vector2f a, final Vector2f b, float radius, SegmentQueryInfo info);

	/**
	 * Segment queries are like ray casting, but because not all spatial indexes allow processing infinitely long ray
	 * queries it is limited to segments. In practice this is still very fast and you don’alpha need to worry too much
	 * about
	 * the performance as long as you aren’alpha using extremely long segments for your queries.
	 * <point/>
	 * Perform a segment query from <b><code>a</code></b> to <b><code>b</code></b> against this shape.
	 * <b><code>info</code></b> must be a valid pointer to a {@link SegmentQueryInfo} structure which will be
	 * initialized
	 * with the raycast info.
	 *
	 * @param a    start of the segment
	 * @param b    end of the segment
	 * @param info the raycast info taking the result
	 * @return <code>true</code> if this shape has been hit by the ray
	 */
	public boolean segmentQuery(final Vector2f a, final Vector2f b, float radius, SegmentQueryInfo info) {
		info.set(null, b, cpvzero(), 1.0f);

		PointQueryInfo nearest;
		nearest = pointQuery(a, null);
		if (nearest.distance <= radius) {
			info.shape = this;
			info.alpha = 0.0f;
			info.normal = cpvnormalize(cpvsub(a, nearest.point));
		} else {
			segmentQueryImpl(a, b, radius, info);
		}

		return (info.shape != null);
	}

	public BB getBB() {
		return bb;
	}

	/** @return the user data */
	public Object getData() {
		return data;
	}

	/**
	 * Sets user data. Use this data to get a reference to the game object that owns this body from callbacks.
	 *
	 * @param data the user data to set
	 */
	public void setData(Object data) {
		this.data = data;
	}

	/**
	 * @param clazz the {@link Class} of the user data
	 * @param <T>   the type of the data
	 * @return the user data
	 */
	public <T> T getData(Class<T> clazz) {
		return clazz.cast(data);
	}

	/**
	 * Nearest point queries return the point on the surface of the shape as well as the distance from the query
	 * point to
	 * the surface point.
	 * <point/>
	 * Find the distance from point <code>point</code> to <code>this</code> shape. If the point is inside of the
	 * shape, the
	 * distance will be negative and equal to the depth of the point.
	 *
	 * @param p   the point for which to find the distance and the nearest point on the surface of the shape
	 * @param out if not <code>null</code> use this object as return value, else a new instance will be created
	 * @return a {@link PointQueryInfo} instance containing the information about the query
	 */
	public abstract PointQueryInfo pointQuery(Vector2f p, PointQueryInfo out);

	protected abstract BB cacheData(Transform transform);

	public BB cacheBB() {
		return update(body.transform);
	}

	public BB update(Transform transform) {
		return (this.bb = cacheData(transform));
	}

	@Override
	public String toString() {
		return "Shape{" +
				"hashid=" + hashid +
				", bb=" + bb +
				", collisionType=" + collisionType +
				'}';
	}
}
