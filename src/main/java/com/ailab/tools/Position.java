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

    public Position(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

   public Position(double pos[])
   {
      this.x = pos[0];
      this.y = pos[1];
       if(pos.length > 2) {
           this.z = pos[2];
       }
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
