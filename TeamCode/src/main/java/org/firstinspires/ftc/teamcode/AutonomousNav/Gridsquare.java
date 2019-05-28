package org.firstinspires.ftc.teamcode.AutonomousNav;

public class Gridsquare {
    /** x coordinate*/
    public int X;
    /** y coordinate*/
    public int Y;
    /** pathfinding weight, higher means less likely to be chosen */
    public int weight;
}

public class Edge {
  public Gridsquare 1;
  public Gridsquare 2;
}
