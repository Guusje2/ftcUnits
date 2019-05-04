package org.firstinspires.ftc.teamcode.AutonomousNav;

public class Vector2 {
    public float X;
    public float Y;

    public void DistanceToVector2(Vector2 b) {
        return Math.sqrt((this.X + b.X)^2 + (this.Y + b.Y)^2);
    }
}
