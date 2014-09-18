package com.ailab.tools;

import com.fasterxml.jackson.annotation.JsonProperty;

/**
 * Created by Patrik on 2014-09-15.
 */
public class PathNode {


    @JsonProperty("Status")
    private int status;
    @JsonProperty("Timestamp")
    private int timestamp;
    @JsonProperty("Pose")
    private Pose pose;

    public Pose getPose() {
        return pose;
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public int getStatus() {
        return status;
    }

    public void setStatus(int status) {
        this.status = status;
    }

    public int getTimestamp() {
        return timestamp;
    }

    public void setTimestamp(int timestamp) {
        this.timestamp = timestamp;
    }

    public String toString() {
        return ("X: " + pose.getPosition().getX() + " Y: " + pose.getPosition().getY());
    }
}
