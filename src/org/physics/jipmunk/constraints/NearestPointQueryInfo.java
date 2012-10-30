package org.physics.jipmunk.constraints;

import org.physics.jipmunk.Shape;
import org.physics.jipmunk.Util;
import org.physics.jipmunk.Vector2f;

/**
 * Nearest point query info struct.
 *
 * @author jobernolte
 */
public class NearestPointQueryInfo {
    /** The nearest shape, NULL if no shape was within range. */
    public Shape shape;
    /** The closest point on the shape's surface. (in world space coordinates) */
    public Vector2f p = Util.cpvzero();
    /** The distance to the point. The distance is negative if the point is inside the shape. */
    public float d;

    public NearestPointQueryInfo() {
    }

    public NearestPointQueryInfo(Shape shape, Vector2f p, float d) {
        this.shape = shape;
        this.p = p;
        this.d = d;
    }

    public void set(Shape shape, Vector2f p, float d) {
        this.shape = shape;
        this.p.set(p);
        this.d = d;
    }

    public void reset() {
        this.shape = null;
        this.p.set(0, 0);
        this.d = 0;
    }
}
