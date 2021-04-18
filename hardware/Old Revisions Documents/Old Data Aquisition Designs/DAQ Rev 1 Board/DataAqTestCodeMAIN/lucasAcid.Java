// Virginia Tech Honor Code Pledge:
//
// As a Hokie, I will conduct myself with honor and integrity at all times.
// I will not lie, cheat, or steal, nor will I accept the actions of those
// who do.
// -- Lucas Nunn (nlucas)
import sofia.micro.*;
import sofia.graphics.Color;

//-------------------------------------------------------------------------
/**
 *  Subclass for acid, which is like water but
 *  is able to dissolve other particles
 *
 *  @author Lucas Nunn (nlucas)
 *  @version (2019.04.01)
 */
public class Acid extends Fluid
{
    //~ Fields ................................................................

    //~ Constructor ...........................................................
    // ----------------------------------------------------------
    /**
     * Creates a new Acid object.
     */
    public Acid()
    {
        super(Color.green, false, 1.0);
    }

    //~ Methods ...............................................................

    /**
     * Dissolves other particles if they can be
     * dissolved.
     * @param x         x position of the particle
     * @param y         y position of the particle
     */
    public void dissolveIfPossible(int x, int y) {
        if (x > 0 && x < getWorld().getWidth() && y > 0 && y < getWorld().getHeight()) {
            Particle jeff = getWorld().getOneObjectAt(x, y, Particle.class);
            if (jeff != null && jeff.willDissolve()) {
                weaken();
                jeff.weaken();
            }
        }
    }

    /**
     * Overrides the act method in the particle
     * class, making the acid particles dissolve
     * other particles
     */
     public void act() {
         for (Particle neighbor : getNeighbors(1, false, Particle.class)) {
             if (getStrength() == 0) {
                 break;
             }
             else {
                 dissolveIfPossible(neighbor.getGridX(), neighbor.getGridY());
             }
         }
         if (getStrength() > 0) {
             super.act();
         }
     }
}
