package com.ailab.controllers;

import com.ailab.tools.Position;

import java.util.*;

/**
 * Created by Jonas on 2014-09-22.
 */
public class VFHPlus {

    public static final double a = 1.0;
    public static final double ROBOT_RADIUS = 0.003;
    public static final double d_s = 0.001;
    public static final double ENLARGEMENT_RADIUS = ROBOT_RADIUS + d_s;
    public static final int NO_OF_SECTORS = 5;
    public static final int READS_PER_SECTOR = 270 / NO_OF_SECTORS;
    public static final double thresholdLow = 0.3;
    public static final double thresholdHigh = 0.7;


    public double getSteeringDirection(Map<Integer, Integer> binaryHistogram, double goalDirection) {
        Object[] objectKeys = binaryHistogram.keySet().toArray();
        List<Integer> sortedKeys = Arrays.asList(Arrays.copyOf(objectKeys, objectKeys.length, Integer[].class));
        Collections.sort(sortedKeys);
        ArrayList<Integer> keys = new ArrayList<Integer>(sortedKeys);
        ArrayList<CandidateDirection> candidateDirections = new ArrayList<CandidateDirection>();

        int index = 0;
        Integer startSector = null;
        while (index < keys.size()) {
            if (binaryHistogram.get(index) == 0) {
                if(startSector == null) {
                    startSector = index;
                }
            } else {
                if(startSector != null) {
                    CandidateDirection candidateDirection = new CandidateDirection(startSector, index - 1);
                    candidateDirections.add(candidateDirection);
                    startSector = null;
                }
            }
            index++;
        }
        if (startSector != null) {
            candidateDirections.add(new CandidateDirection(startSector, keys.size() - 1));
        }





        return 0;
    }

    public static Map<Integer, Integer> buildBinaryHistogram(Map<Integer, Double> histogram) {
        Object[] objectKeys = histogram.keySet().toArray();
        List<Integer> sortedKeys = Arrays.asList(Arrays.copyOf(objectKeys, objectKeys.length, Integer[].class));
        Collections.sort(sortedKeys);
        ArrayList<Integer> keys = new ArrayList<Integer>(sortedKeys);

        Map<Integer, Integer> binaryHistogram = new HashMap<Integer, Integer>();
        for (Integer key : keys) {
            double magnitude = histogram.get(key);
            if(magnitude > thresholdHigh) {
                binaryHistogram.put(key, 1);
            } else if(magnitude < thresholdLow) {
                binaryHistogram.put(key, 0);
            } else {
                Integer previousMagnitude = binaryHistogram.get(key - 1);
                binaryHistogram.put(key, (previousMagnitude == null) ? 0 : previousMagnitude);
            }
        }

        return binaryHistogram;
    }

    public static Map<Integer, Double> buildPolarHistogram(double distanceToCarrotPoint, double distances[], double angle_increment, double start_angle, double vehicle_angle) {

        double d_max = distanceToCarrotPoint;
        Map<Integer, Double> sector_magnitude = new HashMap<Integer, Double>();
        double b = a / d_max;
        double enlargement_angle;
        double direction_to_obstacle;


        for(int i = 0; i < distances.length; i++) {
            if(distances[i] <= d_max) {
                double magnitude = a - (b*distances[i]);
                enlargement_angle = Math.asin(ENLARGEMENT_RADIUS / distances[i]);
                direction_to_obstacle = start_angle + (i*angle_increment);

                int startSector = (int)((direction_to_obstacle - enlargement_angle) / (READS_PER_SECTOR * angle_increment));
                int endSector = (int) ((direction_to_obstacle + enlargement_angle) / (READS_PER_SECTOR * angle_increment)) + 1;
                for(int k = startSector; k < endSector; k++) {
                    if (!sector_magnitude.containsKey(k) || (sector_magnitude.get(k) < magnitude)) {
                        sector_magnitude.put(k, magnitude);
                    }
                }
            }
        }

        return sector_magnitude;
    }

    private class CandidateDirection {

        public static final int sMAX = 4;
        private int leftBorderSector;
        private int rightBorderSector;

        public CandidateDirection (int leftBorderSector, int rightBorderSector) {
            this.leftBorderSector = leftBorderSector;
            this.rightBorderSector = rightBorderSector;
        }

        public void setLeftBorderSector(int leftBorderSector) {
            this.leftBorderSector = leftBorderSector;
        }

        public void setRightBorderSector(int rightBorderSector) {
            this.rightBorderSector = rightBorderSector;
        }

        public int getRightBorderSector () {
            return rightBorderSector;
        }

        public int getLeftBorderSector() {
            return leftBorderSector;
        }

        public boolean isWideValley() {
            return (leftBorderSector - rightBorderSector) > sMAX;
        }
    }
}