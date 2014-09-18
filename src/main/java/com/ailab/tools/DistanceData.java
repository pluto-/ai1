package com.ailab.tools;

/**
 * Created by Jonas on 2014-09-16.
 */
public class DistanceData {

    public static final int P0 = 1;
    public static final int P1 = 2;
    public static final int SEGMENT = 3;

    private double distance;

    public double getDistance() {
        return distance;
    }

    public int getType() {
        return type;
    }

    private int type;

    public DistanceData(double distance, int type) {
        this.distance = distance;
        this.type = type;
    }


}
