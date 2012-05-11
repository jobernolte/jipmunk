package org.physics.jipmunk;

/**
 * Spatial query callback function type.
 *
 * @author jobernolte
 */
interface SpatialReIndexQueryFunc<T> {
	void apply(T obj1, T obj2);
}
