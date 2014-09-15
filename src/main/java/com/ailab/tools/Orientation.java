package com.ailab.tools;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * Created by Patrik on 2014-09-15.
 */
public class Orientation {

    @JsonProperty("W")
    private double w;
    @JsonProperty("X")
    private double x;
    @JsonProperty("Y")
    private double y;
    @JsonProperty("Z")
    private double z;

    public double getW() {
        return w;
    }

    public void setW(double w) {
        this.w = w;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public double getZ() {
        return z;
    }

    public void setZ(double z) {
        this.z = z;
    }

    public void setY(double y) {
        this.y = y;
    }
}
