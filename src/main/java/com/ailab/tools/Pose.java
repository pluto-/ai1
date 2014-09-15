package com.ailab.tools;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * Created by Patrik on 2014-09-15.
 */
public class Pose {

    @JsonProperty("Orientation")
    private Orientation orientation;
    @JsonProperty("Position")
    private Position position;

    public Orientation getOrientation() {
        return orientation;
    }

    public void setOrientation(Orientation orientation) {
        this.orientation = orientation;
    }

    public Position getPosition() {
        return position;
    }

    public void setPosition(Position position) {
        this.position = position;
    }
}
