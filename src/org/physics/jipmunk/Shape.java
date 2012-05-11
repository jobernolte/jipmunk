package org.physics.jipmunk;

import static org.physics.jipmunk.Util.cpvzero;

/** @author jobernolte */
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

	public Shape(Body body) {
		this.body = body;
	}

	public Space getSpace() {
		return space;
	}

	public abstract ShapeType getType();

	public Body getBody() {
		return body;
	}

	public void setBody(Body body) {
		this.body = body;
	}

	public float getElasticity() {
		return e;
	}

	public void setElasticity(float e) {
		this.e = e;
	}

	public float getFrictionCoefficient() {
		return u;
	}

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

	public int getLayers() {
		return layers;
	}

	public void setLayers(int layers) {
		this.layers = layers;
		body.activate();
	}

	public int getGroup() {
		return group;
	}

	public void setGroup(int group) {
		this.group = group;
		body.activate();
	}

	public int getCollisionType() {
		return collision_type;
	}

	public void setCollisionType(int collision_type) {
		this.collision_type = collision_type;
		body.activate();
	}

	public Vector2f getSurfaceVelocity() {
		return surface_v;
	}

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

	public abstract boolean pointQuery(final Vector2f p);

	protected abstract void segmentQueryImpl(final Vector2f a, final Vector2f b, SegmentQueryInfo info);

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
}
