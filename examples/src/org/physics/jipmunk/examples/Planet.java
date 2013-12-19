package org.physics.jipmunk.examples;

import org.physics.jipmunk.*;

import java.util.Random;

import static org.physics.jipmunk.Util.cpv;

/**
 * @author chris_c - converted from chipmunk demo
 */
public class Planet extends ExampleBase {
    private static Space space;
    private Body planetBody;
    static final float gravityStrength = 5.0e6f;
    private static Random rg = new Random();
    private planetGravityVelocityFunc pgvf = new planetGravityVelocityFunc();

    @Override
    public Space init() {
        this.space = new Space();
        space.setGravity(cpv(0, -100));

        Body staticBody = space.getStaticBody();
        Shape shape;
        Body body;

        // Create a rouge body to control the planet manually.
        planetBody = new Body(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
        planetBody.setAngularVelocity(0.2f);

        for (int i = 0; i < 30; i++)
            add_box();

        shape = space.addShape(new CircleShape(planetBody, 70.0f, Util.cpvzero()));
        shape.setElasticity(1.0f);
        shape.setFriction(1.0f);
        shape.setFilter(NOT_GRABABLE_FILTER);

        return space;
    }

    @Override
    public void update(long delta) {

        int steps = 2;
        float dt = 1.0f / 60.0f / (float) steps;

        for (int j = 0; j < steps; j++) {
            space.step(dt);
            // Update the static body spin so that it looks like it's rotating.
            planetBody.updatePosition(dt);
        }
    }


    class planetGravityVelocityFunc implements BodyVelocityFunc {

        public void apply(Body body, Vector2f gravity, float damping, float dt) {
            // Gravitational acceleration is proportional to the inverse square of
            // distance, and directed toward the origin. The central planet is assumed
            // to be massive enough that it affects the satellites but not vice versa.
            Vector2f p = body.getPosition();
            float sqdist = Util.cpvlengthsq(p);
            Vector2f g = Util.cpvmult(p, -gravityStrength / (sqdist * Util.cpfsqrt(sqdist)));

            body.updateVelocity(g, damping, dt);
        }
    }

    private static Vector2f rand_pos(float radius) {
        Vector2f v;
        do {
            v = Util.cpv(rg.nextFloat() * (640f - 2f * radius) - (320f - radius),
                    rg.nextFloat() * (480f - 2f * radius) - (240f - radius));
        } while (Util.cpvlength(v) < 85.0f);

        return v;
    }

    private void add_box() {
        final float size = 10.0f;
        final float mass = 1.0f;

        Vector2f[] verts = {
                Util.cpv(-size, -size),
                Util.cpv(-size, size),
                Util.cpv(size, size),
                Util.cpv(size, -size),
        };

        float radius = Util.cpvlength(Util.cpv(size, size));
        Vector2f pos = rand_pos(radius);

        Body body = space.addBody(new Body(mass, Util.momentForPoly(mass, verts, Util.cpvzero(), 0.0f)));
        body.setVelocityFunc(pgvf);
        body.setPosition(pos);

        // Set the box's velocity to put it into a circular orbit from its
        // starting position.
        float r = Util.cpvlength(pos);
        float v = Util.cpfsqrt(gravityStrength / r) / r;
        body.setVelocity(Util.cpvmult(Util.cpvperp(pos), v));

        // Set the box's angular velocity to match its orbital period and
        // align its initial angle with its position.
        body.setAngularVelocity(v);
        body.setAngleInRadians(Util.cpvtoangle(pos));
        Shape shape = space.addShape(new PolyShape(body, 0.0f, verts));
        shape.setElasticity(0.0f);
        shape.setFriction(0.7f);
    }


    public static void main(String[] args) {
        new Planet().start(640, 480);
    }
}
