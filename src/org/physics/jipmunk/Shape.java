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

import org.physics.jipmunk.constraints.NearestPointQueryInfo;

import static org.physics.jipmunk.Util.cpvzero;

/**
 * There are currently 3 collision shape types:
 * <p/>
 * <ul> <li><b>Circles</b>: Fastest and simplest collision shape.</li> <li><b>Line segments</b>: Meant mainly as a
 * static shape. They can be attached to moving bodies, but they don’t currently generate collisions with other line
 * segments. Can be beveled in order to give them a thickness.</li> <li><b>Convex polygons</b>: Slowest, but most
 * flexible collision shape.</li> </ul>
 * <p/>
 * You can add as many shapes to a body as you wish. That is why the two types are separate. This should give you the
 * flexibility to make any shape you want as well providing different areas of the same object with different friction,
 * elasticity or callback values.
 *
 * @author jobernolte
 */
public abstract class Shape {

	int hashid = -1;

	/** The rigid body this collision shape is attached to. */
	Body body;

	/** The current bounding box of the shape. */
	protected BB bb;

	/** Sensor flag. Sensor shapes call collision callbacks but don't produce collisions. */
	boolean sensor = false;

	/** Coefficient of restitution. (elasticity) */
	float e = 0;
	/** Coefficient of friction. */
	float u = 0;
	/** Surface velocity used when solving for friction. */
	Vector2f surface_v = cpvzero();

	/** Collision type of this shape used when picking collision handlers. */
	int collision_type = 0;
	/** Group of this shape. Shapes in the same group don't collide. */
	int group = Constants.NO_GROUP;
	/** Layer bitmask for this shape. Shapes only collide if the bitwise and of their layers is non-zero. */
	int layers = Constants.ALL_LAYERS;

	Shape prev;
	Shape next;
	Space space;

	/**
	 * User definable data. Generally this points to your the game object class so you can access it when given a Body
	 * reference in a callback.
	 */
	private Object data;

	public Shape(Body body) {
		this.body = body;
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

	/** @return the elasticity of the shape */
	public float getElasticity() {
		return e;
	}

	/**
	 * Elasticity of the shape. A value of 0.0 gives no bounce, while a value of 1.0 will give a “perfect” bounce. However
	 * due to inaccuracies in the simulation using 1.0 or greater is not recommended however. The elasticity for a
	 * collision is found by multiplying the elasticity of the individual shapes together.
	 *
	 * @param e the elasticity of the shape
	 */
	public void setElasticity(float e) {
		this.e = e;
	}

	/** @return the friction coefficient of this shape */
	public float getFrictionCoefficient() {
		return u;
	}

	/**
	 * Friction coefficient. Chipmunk uses the Coulomb friction model, a value of 0.0 is frictionless. The friction for a
	 * collision is found by multiplying the friction of the individual shapes together.
	 * <p/>
	 * Tables of friction coefficients: http://www.roymech.co.uk/Useful_Tables/Tribology/co_of_frict.htm
	 *
	 * @param u the friction coefficient
	 */
	public void setFrictionCoefficient(float u) {
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

	/** @return the layer bitmask */
	public int getLayers() {
		return layers;
	}

	/**
	 * Shapes only collide if they are in the same bit-planes. i.e. (a->layers & b->layers) != 0 By default, a shape
	 * occupies all bit-planes. Wikipedia has a nice article on bitmasks if you are unfamiliar with how to use them.
	 * Defaults to {@link Constants#ALL_LAYERS}.
	 *
	 * @param layers the layer bitmask
	 */
	public void setLayers(int layers) {
		this.layers = layers;
		body.activate();
	}

	/** @return the collision group of this shape */
	public int getGroup() {
		return group;
	}

	/**
	 * Shapes in the same non-zero group do not generate collisions. Useful when creating an object out of many shapes that
	 * you don’t want to self collide. Defaults to {@link Constants#NO_GROUP}.
	 *
	 * @param group the collision group of this shape
	 */
	public void setGroup(int group) {
		this.group = group;
		body.activate();
	}

	/** @return the collision type of this shape */
	public int getCollisionType() {
		return collision_type;
	}

	/**
	 * You can assign types to Chipmunk collision shapes that trigger callbacks when objects of certain types touch. See
	 * the callbacks section for more information.
	 *
	 * @param collision_type the collision type
	 */
	public void setCollisionType(int collision_type) {
		this.collision_type = collision_type;
		body.activate();
	}

	/** @return the surface velocity of this shape */
	public Vector2f getSurfaceVelocity() {
		return surface_v;
	}

	/**
	 * The surface velocity of the object. Useful for creating conveyor belts or players that move around. This value is
	 * only used when calculating friction, not resolving the collision.
	 *
	 * @param surface_v the surface velocity
	 */
	public void setSurfaceVelocity(Vector2f surface_v) {
		this.surface_v = surface_v;
		body.activate();
	}

	BB cacheBB() {
		return update(body.getPosition(), body.getRotation());
	}

	BB update(final Vector2f pos, final Vector2f rot) {
		return (bb = cacheData(pos, rot));
	}

	protected abstract BB cacheData(Vector2f pos, Vector2f rot);

	/**
	 * Check if the given point lies within the shape.
	 *
	 * @param p the point to check
	 * @return <code>true</code> if the point lies within this shape
	 */
	public abstract boolean pointQuery(final Vector2f p);

	protected abstract void segmentQueryImpl(final Vector2f a, final Vector2f b, SegmentQueryInfo info);

	/**
	 * Segment queries are like ray casting, but because not all spatial indexes allow processing infinitely long ray
	 * queries it is limited to segments. In practice this is still very fast and you don’t need to worry too much about
	 * the performance as long as you aren’t using extremely long segments for your queries.
	 * <p/>
	 * Perform a segment query from <b><code>a</code></b> to <b><code>b</code></b> against this shape.
	 * <b><code>info</code></b> must be a valid pointer to a {@link SegmentQueryInfo} structure which will be initialized
	 * with the raycast info.
	 *
	 * @param a    start of the segment
	 * @param b    end of the segment
	 * @param info the raycast info taking the result
	 * @return <code>true</code> if this shape has been hit by the ray
	 */
	public boolean segmentQuery(final Vector2f a, final Vector2f b, SegmentQueryInfo info) {
		info.shape = null;
		info.t = 0;
		info.n = cpvzero();
		segmentQueryImpl(a, b, info);
		return info.shape != null;
	}

	static BB cpShapeUpdate(Shape shape, final Vector2f pos, final Vector2f rot) {
		return shape.update(pos, rot);
	}

	static boolean cpShapePointQuery(Shape shape, final Vector2f p) {
		return shape.pointQuery(p);
	}

	static boolean cpShapeSegmentQuery(Shape shape, final Vector2f a, final Vector2f b, SegmentQueryInfo info) {
		return shape.segmentQuery(a, b, info);
	}

	public BB getBb() {
		return bb;
	}

	/** @return the user data */
	public Object getData() {
		return data;
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
	 * Sets user data. Use this data to get a reference to the game object that owns this body from callbacks.
	 *
	 * @param data the user data to set
	 */
	public void setData(Object data) {
		this.data = data;
	}

	/**
	 * Nearest point queries return the point on the surface of the shape as well as the distance from the query point to
	 * the surface point.
	 * <p/>
	 * Find the distance from point <code>p</code> to <code>this</code> shape. If the point is inside of the shape, the
	 * distance will be negative and equal to the depth of the point.
	 *
	 * @param p   the point for which to find the distance and the nearest point on the surface of the shape
	 * @param out if not <code>null</code> use this object as return value, else a new instance will be created
	 * @return a {@link NearestPointQueryInfo} instance containing the information about the query
	 */
	public abstract NearestPointQueryInfo nearestPointQuery(Vector2f p, NearestPointQueryInfo out);
}
