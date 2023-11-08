package org.firstinspires.ftc.teamcode.util;

public class Vector2d<T extends Number> {
    protected T _x,_y;
    public Vector2d(T x, T y) {
        _x = x;
        _y = y;
    }
    public T getX() {return _x;};
    public T getY() {return _y;};
    public Vector2d add(Vector2d v) {
        return sum(this,v);
    }

    public double getLength() {
        if (_x instanceof Integer) {
            return Math.sqrt(1.0*(_x.intValue()*_x.intValue() + _y.intValue()*_y.intValue()));
        }
        if (_x instanceof Double) {
            return Math.sqrt(1.0*(_x.doubleValue()*_x.doubleValue() + _y.doubleValue()*_y.doubleValue()));
        }
        return 0.0;
    }

    public static Vector2d sum(Vector2d u, Vector2d v) {
        if (u._x instanceof Integer) {
            return new Vector2d( Integer.valueOf(u._x.intValue() + v._x.intValue()), Integer.valueOf(u._y.intValue() + v._y.intValue()));
        }
        if (u._x instanceof Double) {
            return new Vector2d( Double.valueOf(u._x.doubleValue() + v._x.doubleValue()), Double.valueOf(u._y.doubleValue() + v._y.doubleValue()));
        }
        return null;
    }
}
