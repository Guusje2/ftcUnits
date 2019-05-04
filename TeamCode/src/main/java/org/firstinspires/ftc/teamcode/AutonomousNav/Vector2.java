package org.firstinspires.ftc.teamcode.AutonomousNav;

public class Vector2 {
    public double X;
    public double Y;

    public double DistanceToVector2(Vector2 b) {
        return Math.sqrt((this.X - b.X)*(this.X - b.X) + (this.Y - b.Y)*(this.Y - b.Y));
    }


}
