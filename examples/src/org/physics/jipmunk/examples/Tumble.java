package org.physics.jipmunk.examples;

import org.physics.jipmunk.*;

import static org.physics.jipmunk.Util.cpv;

/**
 * @author chris_c - converted from chipmunk demo
 */
public class Tumble extends ExampleBase {
    private Space space;
    private Body rogueBoxBody;

    @Override
    public Space init() {
        this.space = new Space();
        space.setGravity(cpv(0, -600));

        Body staticBody = space.getStaticBody();
        Shape shape;
        Body body;

        // We create an infinite mass rogue body to attach the line segments too
        // This way we can control the rotation however we want.
        rogueBoxBody = new Body(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
        rogueBoxBody.setAngVel(0.4f);

        // Set up the static box.
        Vector2f a = cpv(-200, -200);
        Vector2f b = cpv(-200, 200);
        Vector2f c = cpv(200, 200);
        Vector2f d = cpv(200, -200);

        shape = space.addShape(new SegmentShape(rogueBoxBody, a, b, 0.0f));
        shape.setElasticity(1.0f);
        shape.setFrictionCoefficient(1.0f);
        shape.setLayers(NOT_GRABABLE_MASK);

        shape = space.addShape(new SegmentShape(rogueBoxBody, b, c, 0.0f));
        shape.setElasticity(1.0f);
        shape.setFrictionCoefficient(1.0f);
        shape.setLayers(NOT_GRABABLE_MASK);

        shape = space.addShape(new SegmentShape(rogueBoxBody, c, d, 0.0f));
        shape.setElasticity(1.0f);
        shape.setFrictionCoefficient(1.0f);
        shape.setLayers(NOT_GRABABLE_MASK);

        shape = space.addShape(new SegmentShape(rogueBoxBody, d, a, 0.0f));
        shape.setElasticity(1.0f);
        shape.setFrictionCoefficient(1.0f);
        shape.setLayers(NOT_GRABABLE_MASK);


        float mass = 1f;
        float width = 60f;
        float height = 30f;

        // Add the bricks.
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 7; j++) {
                body = space.addBody(new Body(mass, Util.momentForBox(mass, width, height)));
                body.setPosition(cpv(i * 60 - 150, j * 30 - 150));

                shape = space.addShape(PolyShape.createBox(body, width, height));
                shape.setElasticity(0.0f);
                shape.setFrictionCoefficient(0.7f);

            }
        }

        return space;
    }

    @Override
    public void update(long delta) {

        int steps = 2;
        float dt = 1.0f / 60.0f / (float) steps;

        for (int j = 0; j < steps; j++) {
            Body.updatePosition(rogueBoxBody, dt);
            space.step(dt);
        }
    }

    public static void main(String[] args) {
        new Tumble().start(640, 480);
    }
}