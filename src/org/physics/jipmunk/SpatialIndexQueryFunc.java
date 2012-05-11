package org.physics.jipmunk;

/**
 * Spatial query callback function type.
 *
 * @author jobernolte
 */
interface SpatialIndexQueryFunc<T> {
	void apply(T obj2);
}
