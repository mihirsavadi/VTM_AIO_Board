import sofia.micro.*;
import sofia.graphics.Color;

//-------------------------------------------------------------------------
/**
 *  Write a one-sentence summary of your class here.
 *  Follow it with additional details about its purpose, what abstraction
 *  it represents, and how to use it.
 *
 *  @author your name (your-pid)
 *  @version (place the date here, in this format: yyyy.mm.dd)
 */
public class Particle extends ParticleBase
{
    //~ Fields ................................................................

    private double velocity;
    private double acceleration;
    private int strength;
    private double densityValue;
    private boolean dissolvability;

    //~ Constructor ...........................................................
    // ----------------------------------------------------------
    /**
     * Creates a new Particle object.
     */
    public Particle(Color color, boolean willDissolve, double density)
    {
        super(color);
        velocity = 0.0;
        acceleration = 1.0;
        strength = 100;
        densityValue = density;
        dissolvability = willDissolve;
    }

    //~ Methods ...............................................................

    public double getDensity() {
        return densityValue;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public int getStrength() {
        return strength;
    }

    public boolean willDissolve() {
        return dissolvability;
    }

    public void weaken() {
        if (strength == 0) {
            remove();
        }
        else {
            strength = strength - 1;
        }
    }

    public boolean isFalling() {
        if (getGridY() < getWorld().getHeight() - 1) {
            int a = getGridX();
            int b = getGridY() + 1;
            Particle timmy = getWorld().getOneObjectAt(a, b, Particle.class);
            if (timmy == null) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }

    public void fall() {
        velocity = velocity + acceleration;
        int distance = (int) (velocity);
        for (int i = 0; i < distance; i++) {
            if (isFalling()) {
                setGridY(getGridY() + 1);
            }
            else {
                velocity = 0;
            }
        }
    }

    public boolean swapPlacesIfPossible(int x, int y) {
        if (x < getWorld().getWidth() && x >= 0 && y < getWorld().getHeight() && y >= 0) {
            int a = getGridX();
            int b = getGridY();
            Particle pete = getWorld().getOneObjectAt(x, y, Particle.class);
            if (pete == null) {
                setGridLocation(x, y);
                return true;
            }
            else {
                if (getDensity() > pete.getDensity()) {
                    pete.setGridLocation(a, b);
                    setGridLocation(x, y);
                    return true;
                }
                else {
                    return false;
                }
            }
        }
        return false;
    }

    public boolean dodge() {
        int a = getGridX();
        int b = getGridY();
        swapPlacesIfPossible(a, b + 1);
        if (getGridX() != a || getGridY() != b) {
            return true;
        }
        swapPlacesIfPossible(a - 1, b + 1);
        if (getGridX() != a || getGridY() != b) {
            return true;
        }
        swapPlacesIfPossible(a + 1, b + 1);
        if (getGridX() != a || getGridY() != b) {
            return true;
        }
        return false;
    }

    public void act() {
        if (isFalling()) {
            fall();
        }
        else {
            dodge();
        }
    }
}
