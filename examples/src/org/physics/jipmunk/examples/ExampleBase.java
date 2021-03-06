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

package org.physics.jipmunk.examples;

import java.awt.*;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.BitSet;
import java.util.LinkedList;
import java.util.List;
import javax.media.opengl.GL2;
import javax.media.opengl.GLAutoDrawable;
import javax.media.opengl.GLCapabilities;
import javax.media.opengl.GLEventListener;
import javax.media.opengl.GLProfile;
import javax.media.opengl.awt.GLCanvas;
import javax.media.opengl.glu.GLU;

import org.physics.jipmunk.*;
import org.physics.jipmunk.constraints.PivotJoint;
import com.jogamp.opengl.util.FPSAnimator;
import com.jogamp.opengl.util.awt.TextRenderer;

/**
 * @author jobernolte
 */
public abstract class ExampleBase implements GLEventListener {
    public static final Bitmask GRABABLE_MASK_BIT = new Bitmask(1 << 31);
    public static final Bitmask NOT_GRABABLE_MASK = new Bitmask(~GRABABLE_MASK_BIT.getValue());
    public static final ShapeFilter GRAB_FILTER =
            new ShapeFilter(Constants.NO_GROUP, GRABABLE_MASK_BIT, GRABABLE_MASK_BIT);
    public static final ShapeFilter NOT_GRABABLE_FILTER =
            new ShapeFilter(Constants.NO_GROUP, NOT_GRABABLE_MASK, NOT_GRABABLE_MASK);
    private DrawSpace.Options drawSpaceOptions = new DrawSpace.Options(false, false, true, 6.0f, 4.0f, 1.5f);
    private GL2 gl;
    private GLU glu;
    private DrawSpace drawSpace;
    protected Vector2f mousePoint = Util.cpvzero();
    protected boolean chipmunkDemoRightClick = false;
    protected boolean chipmunkDemoRightDown = false;
    protected Vector2f chipmunkDemoKeyboard = Util.cpvzero();
    private Vector2f mousePoint_last = Util.cpvzero();
    private Body mouseBody = null;
    private Constraint mouseJoint = null;
    private int width;
    private int height;
    private Space space;
    private String messageString;
    private TextRenderer textRenderer;
    private GLCapabilities glCapabilities;
    private GLCanvas glCanvas;
    private Frame frame;
    private long lastUpdate;
    private IntBuffer view = BufferUtils.createIntBuffer(4);
    private FloatBuffer model = BufferUtils.createFloatBuffer(16);
    private FloatBuffer proj = BufferUtils.createFloatBuffer(16);
    private FloatBuffer m = BufferUtils.createFloatBuffer(3);
    private final List<MouseEvent> mouseEvents = new LinkedList<>();
    private final List<KeyEvent> keyEvents = new LinkedList<>();
    private BitSet pressedKeys = new BitSet();

    public abstract Space init();

    public abstract void update(long delta);

    public DrawSpace.Options getDrawSpaceOptions() {
        return drawSpaceOptions;
    }

    public int getHeight() {
        return height;
    }

    public int getWidth() {
        return width;
    }

    public void start(int width, int height) {
        GLProfile glProfile = GLProfile.getDefault();
        glCapabilities = new GLCapabilities(glProfile);
        glCapabilities.setPBuffer(true);
        glCapabilities.setOnscreen(true);
        glCanvas = new GLCanvas(glCapabilities);
        //glCanvas.setContextCreationFlags(GLContext.CTX_OPTION_DEBUG);
        frame = new Frame("AWT");
        frame.setSize(width, height);
        frame.add(glCanvas);
        frame.setVisible(true);
        frame.addWindowListener(new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
                System.exit(0);
            }
        });
        glCanvas.setFocusable(true);
        glCanvas.addGLEventListener(this);
        glCanvas.requestFocus();
    }

    public void exit() {
        frame.dispose();
    }

    @Override
    public void init(GLAutoDrawable glAutoDrawable) {
        glAutoDrawable.getGL().setSwapInterval(1);
        this.gl = glAutoDrawable.getGL().getGL2();
        this.glu = new GLU();
        this.textRenderer = new TextRenderer(new Font("SansSerif", 0, 14));
        FPSAnimator animator = new FPSAnimator(60);
        animator.add(glAutoDrawable);
        animator.start();
        this.width = glAutoDrawable.getWidth();
        this.height = glAutoDrawable.getHeight();
        this.space = init();
        this.mouseBody = new Body(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY);
        this.drawSpaceOptions = getDrawSpaceOptions();
        this.drawSpace = new DrawSpace(this.gl);

        glCanvas.addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                synchronized (mouseEvents) {
                    mouseEvents.add(e);
                }
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                synchronized (mouseEvents) {
                    mouseEvents.add(e);
                }
            }

            @Override
            public void mouseMoved(MouseEvent e) {
                synchronized (mouseEvents) {
                    mouseEvents.add(e);
                }
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                synchronized (mouseEvents) {
                    mouseEvents.add(e);
                }
            }
        });
        glCanvas.addMouseMotionListener(new MouseAdapter() {
            @Override
            public void mouseMoved(MouseEvent e) {
                synchronized (mouseEvents) {
                    mouseEvents.add(e);
                }
            }

            @Override
            public void mouseDragged(MouseEvent e) {
                synchronized (mouseEvents) {
                    mouseEvents.add(e);
                }
            }
        });
        glCanvas.addKeyListener(new KeyAdapter() {
            @Override
            public void keyPressed(KeyEvent e) {
                synchronized (keyEvents) {
                    keyEvents.add(e);
                }
            }

            @Override
            public void keyReleased(KeyEvent e) {
                synchronized (keyEvents) {
                    keyEvents.add(e);
                }
            }
        });
    }

    @Override
    public void dispose(GLAutoDrawable glAutoDrawable) {
    }

    private MouseEvent pollNextMouseEvent() {
        MouseEvent mouseEvent = null;
        synchronized (mouseEvents) {
            if (mouseEvents.size() > 0) {
                mouseEvent = mouseEvents.remove(0);
            }
        }
        return mouseEvent;
    }

    private KeyEvent pollNextKeyEvent() {
        KeyEvent keyEvent = null;
        synchronized (keyEvents) {
            if (keyEvents.size() > 0) {
                keyEvent = keyEvents.remove(0);
            }
        }
        return keyEvent;
    }

    private void pollEvents() {
        for (MouseEvent e = pollNextMouseEvent(); e != null; e = pollNextMouseEvent()) {
            switch (e.getID()) {
                case MouseEvent.MOUSE_PRESSED: {
                    if (e.getButton() == MouseEvent.BUTTON1) {
                        Vector2f point = mouseToSpace(e.getX(), e.getY());
                        // give the mouse click a little radius to make it easier to click small shapes.
                        float radius = 5.0f;

                        org.physics.jipmunk.Shape shape =
                                space.pointQueryNearest(point, radius, GRAB_FILTER, null).shape;
                        if (shape != null) {
                            Body body = shape.getBody();
                            mouseJoint = new PivotJoint(mouseBody, body, Util.cpvzero(), body.worldToLocal(point));
                            mouseJoint.setMaxForce(50000.0f);
                            mouseJoint.setErrorBias(Util.cpfpow(1.0f - 0.15f, 60.0f));
                            space.addConstraint(mouseJoint);
                        }
                        mousePoint.set(point);
                    }
                    if (e.getButton() == MouseEvent.BUTTON3) {
                        Vector2f point = mouseToSpace(e.getX(), e.getY());
                        chipmunkDemoRightClick = true;
                        chipmunkDemoRightDown = true;
                        mousePoint.set(point);
                    }
                    break;
                }
                case MouseEvent.MOUSE_RELEASED: {
                    if (mouseJoint != null) {
                        space.removeConstraint(mouseJoint);
                        mouseJoint = null;
                    }
                    chipmunkDemoRightClick = false;
                    chipmunkDemoRightDown = false;
                    break;
                }
                case MouseEvent.MOUSE_MOVED:
                case MouseEvent.MOUSE_DRAGGED: {
                    Vector2f p = mouseToSpace(e.getX(), e.getY());
                    mousePoint.set(p);
                    break;
                }
            }
        }
        mouseEvents.clear();
        for (KeyEvent e = pollNextKeyEvent(); e != null; e = pollNextKeyEvent()) {
            switch (e.getID()) {
                case KeyEvent.KEY_PRESSED:
                case KeyEvent.KEY_RELEASED:
                    pressedKeys.set(e.getKeyCode(), e.getID() == KeyEvent.KEY_PRESSED);
                    setArrowDirection();
                    break;
            }
        }
        keyEvents.clear();
    }

    private void setArrowDirection() {
        int x = 0, y = 0;

        if (pressedKeys.get(KeyEvent.VK_UP))
            y += 1;
        if (pressedKeys.get(KeyEvent.VK_DOWN))
            y -= 1;
        if (pressedKeys.get(KeyEvent.VK_RIGHT))
            x += 1;
        if (pressedKeys.get(KeyEvent.VK_LEFT))
            x -= 1;

        chipmunkDemoKeyboard.set(x, y);
    }

    @Override
    public void display(GLAutoDrawable glAutoDrawable) {
        pollEvents();

        Vector2f newPoint = Util.cpvlerp(mouseBody.getPosition(), mousePoint, 0.25f);
        mouseBody.setVelocity(Util.cpvmult(Util.cpvsub(newPoint, mouseBody.getPosition()), 60.0f));
        mouseBody.setPosition(newPoint);

        long now = System.currentTimeMillis();
        if (lastUpdate != 0) {
            update(now - lastUpdate);
        }
        lastUpdate = now;

        //renderer.init();
        //renderer.enterOrtho(640, 480);
        gl.glViewport(0, 0, width, height);

        int rx = (int) (width / 2.0f);
        int ry = (int) (height / 2.0f);

        gl.glMatrixMode(GL2.GL_PROJECTION);
        gl.glLoadIdentity();
        gl.glOrtho(-rx, rx, -ry, ry, -1, 1);
        gl.glTranslatef(0.5f, 0.5f, 0.0f);
        gl.glMatrixMode(GL2.GL_MODELVIEW);
        gl.glLoadIdentity();
        gl.glEnableClientState(GL2.GL_VERTEX_ARRAY);
        gl.glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        gl.glClear(GL2.GL_COLOR_BUFFER_BIT);
        drawSpace.drawSpace(space, drawSpaceOptions);
        gl.glDisableClientState(GL2.GL_VERTEX_ARRAY);

        gl.glGetFloatv(GL2.GL_MODELVIEW_MATRIX, model);
        gl.glGetFloatv(GL2.GL_PROJECTION_MATRIX, proj);
        gl.glGetIntegerv(GL2.GL_VIEWPORT, view);

        if (messageString != null && !messageString.isEmpty()) {
            gl.glMatrixMode(GL2.GL_PROJECTION);
            gl.glPushMatrix();
            gl.glMatrixMode(GL2.GL_MODELVIEW);

            textRenderer.beginRendering(width, height);
            textRenderer.setColor(Color.black);
            textRenderer.draw(messageString, 0, 15);
            textRenderer.endRendering();

            gl.glMatrixMode(GL2.GL_PROJECTION);
            gl.glPopMatrix();
            gl.glMatrixMode(GL2.GL_MODELVIEW);
        }
    }

    @Override
    public void reshape(GLAutoDrawable glAutoDrawable, int x, int y, int width, int height) {
        this.width = width;
        this.height = height;
        gl.glGetIntegerv(GL2.GL_VIEWPORT, view);
    }

    private Vector2f mouseToSpace(int x, int y) {
        proj.rewind();
        model.rewind();
        view.rewind();
        m.rewind();

        glu.gluUnProject(x, height - y, 0.0f, model, proj, view, m);
        m.rewind();
        float mx = m.get();
        float my = m.get();

        return Util.cpv(mx, my);
    }
}
