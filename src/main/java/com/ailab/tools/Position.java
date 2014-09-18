package com.ailab.tools;

import com.fasterxml.jackson.annotation.JsonProperty;

public class Position
{
    @JsonProperty("X")
   private double x;
    @JsonProperty("Y")
   private double y;
    @JsonProperty("Z")
   private double z;

    public Position() {

    }

    public Position(double x, double y) {
        this.x = x;
        this.y = y;
    }

   public Position(double pos[])
   {
      this.x = pos[0];
      this.y = pos[1];
   }

    public void setX(double x) {
        this.x = x;
    }

    public void setY(double y) {
        this.y = y;
    }

    public void setZ(double z) {
        this.z = z;
    }

   public double getX() { return x; }

    public double getY() { return y; }

    public double getZ() {
        return z;
    }

    public double[] getXY() {
        double xy[] = new double[2];
        xy[0] = x;
        xy[1] = y;

        return xy;
    }

   public double getDistanceTo(Position p)
   {
      return Math.sqrt((x - p.x) * (x - p.x) + (y - p.y) * (y - p.y));
   }

   // bearing relative 'north'
   public double getBearingTo(Position p)
   {
      return Math.atan2(p.y - y, p.x - x);
   }
}
