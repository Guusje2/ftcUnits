package org.firstinspires.ftc.teamcode.MathEssentials;

public class Vector2 {
    public double X;
    public double Y;

    public double DistanceToVector2(Vector2 b) {
        return Math.sqrt((this.X - b.X)*(this.X - b.X) + (this.Y - b.Y)*(this.Y - b.Y));
    }
    
    public Vector2 subtract (Vector2 b) {
     Vector2 a = new Vector2();
        a.X = this.X - b.X;
        a.Y = this.Y - b.Y;
        return a;
    }
    
    public Vector2 add (Vector2 b) {
     Vector2 a = new Vector2();
        a.X = this.X + b.X;
        a.Y = this.Y +b.Y;
        return a;
    }
    


}
