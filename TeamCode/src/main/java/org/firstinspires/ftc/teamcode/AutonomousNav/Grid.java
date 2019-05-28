package org.firstinspires.ftc.teamcode.AutonomousNav;

import org.firstinspires.ftc.teamcode.MathEssentials.Vector2;

import java.util.ArrayList;
import java.util.Dictionary;

public class Grid {
    public Dictionary<Vector2, Gridsquare> grid;


    /**
     * gets the neighbours of a gridsquare */

    public Gridsquare[] GetNeighbours (Gridsquare a) {
        ArrayList<Gridsquare> b = new ArrayList<Gridsquare>();
        for (int x = a.X -1; x < a.X + 2; x++ ){
            for (int y = a.Y -1; y < a.Y + 2; y++ ){
                Vector2 c = new Vector2();
                c.X = x;
                c.Y = y;
                b.add(grid.get(c));
            }
        }
        return b.toArray(new Gridsquare[b.size()]);
    }

    /** Creates an grid with a weight of 1 */
    public void CreateGrid (int sizex, int sizey) {
        for (int x = sizex -1; x < sizex-1; x++) {
            for (int y = sizey -1; y < sizey - 1; y++ ){
                Gridsquare a = new Gridsquare();
                a.X = x;
                a.Y = y;
                a.weight = 1;
                Vector2 b = new Vector2();
                b.X = x;
                b.Y = y;
                grid.put(b,a);
            }
        }
    }
}
