package org.physics.jipmunk;

/**
 * Spatial segment query callback function type.
 *
 * @author jobernolte
 */
interface SpatialIndexSegmentQueryFunc<T> {
	float apply(T obj2);
}
