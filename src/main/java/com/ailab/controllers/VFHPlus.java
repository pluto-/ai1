package com.ailab.controllers;

import java.util.*;

/**
 * Created by Jonas on 2014-09-22.
 */
public class VFHPlus {

    public static final double a = 1.0;
    public static final double ROBOT_RADIUS = 0.25;
    public static final double d_s = 0.05;
    public static final double ENLARGEMENT_RADIUS = ROBOT_RADIUS + d_s;
    public static final int NO_OF_SECTORS = 90;
    public static final int READS_PER_SECTOR = 270 / NO_OF_SECTORS;
    public static final int NO_OF_BLIND_SECTORS = (int)(NO_OF_SECTORS / 3);
    public static final double thresholdLow = 0.2;
    public static final double thresholdHigh = 0.5;
    public static final int sMAX = 5;

    public static final double MU1 = 0.8;
    public static final double MU2 = 0.4;
    public static final double MU3 = 0.2;

    private double startAngle;
    private double endAngle;
    private double angleIncrement;
    private double angularResolution;
    private Integer previousTargetSector;

    public VFHPlus (double startAngle, double endAngle, double angleIncrement) {
        this.startAngle = startAngle;
        this.endAngle = endAngle;
        this.angleIncrement = angleIncrement;
        angularResolution = READS_PER_SECTOR * angleIncrement;

        previousTargetSector = null;
    }

    public void resetPreviousTargetSector() {
        previousTargetSector = null;
    }

    public Double calculateSteeringDirection(double distanceToCarrotPoint, double distances[], double goalAngle) {
        Map<Integer, Integer> binaryHistogram = buildBinaryHistogram(buildPolarHistogram(distanceToCarrotPoint, distances));
        if (binaryHistogram == null) {
            return goalAngle;
        }

        return findBestDirection(binaryHistogram, goalAngle);
    }

    private Double findBestDirection(Map<Integer, Integer> binaryHistogram, double goalAngle) {
        int goalSector;
        if(goalAngle < startAngle) {
            goalAngle = startAngle;
        } else if(goalAngle > endAngle) {
            goalAngle = endAngle;
        }
        goalSector = (int)((goalAngle) / angularResolution);

        ArrayList<CandidateValley> candidateValleys = getCandidateValleys(binaryHistogram);
        ArrayList<Integer> candidateDirections = getCandidateDirections(candidateValleys, goalSector);

        double bestCost = Double.MAX_VALUE;
        Integer bestDirection = null;
        for (int i = 0; i < candidateDirections.size(); i++) {
            double candidateCost = calculateCost(candidateDirections.get(i), goalSector, MU1, MU2, MU3);
            if(candidateCost < bestCost) {
                bestCost = candidateCost;
                bestDirection = candidateDirections.get(i);
            }
        }
        previousTargetSector = bestDirection;

        return bestDirection == null ? null : (bestDirection) * angularResolution;
    }


    //4.2.17 https://www8.cs.umu.se/kurser/5DV121/HT14/utdelat/Ringdahl%202003%20Master%20thesis.pdf
    private double calculateCost(int candidateSector, int goalSector, double mu1, double mu2, double mu3) {

        double first = mu1*(calcDeltaSectors(candidateSector, goalSector));
        double second = mu2 * calcDeltaSectors(candidateSector, 0);
        double third = (previousTargetSector == null ? 0 : mu3 * calcDeltaSectors(candidateSector, previousTargetSector));

        return first + second + third;
    }

    public static int calcDeltaSectors(int sectorTwo, int sectorOne) {
        int first = Math.abs(sectorOne - sectorTwo);
        int second = Math.abs(sectorOne - sectorTwo - (NO_OF_SECTORS + NO_OF_BLIND_SECTORS));
        int third = Math.abs(sectorOne - sectorTwo + (NO_OF_SECTORS + NO_OF_BLIND_SECTORS));
        int min_first_second = Math.min(first, second);
        return Math.min(min_first_second, third);
    }

    private ArrayList<Integer> getCandidateDirections(ArrayList<CandidateValley> candidateValleys, int goalSector) {

        ArrayList<Integer> candidateDirections = new ArrayList<Integer>();
        for (CandidateValley candidateValley: candidateValleys) {
            int leftBorderSector = candidateValley.getLeftBorderSector();
            int rightBorderSector = candidateValley.getRightBorderSector();
            if (candidateValley.isWideValley(sMAX)) {
                candidateDirections.add(leftBorderSector - sMAX/2);
                candidateDirections.add(rightBorderSector + sMAX/2);
                if (leftBorderSector >= goalSector && rightBorderSector <= goalSector) {
                    candidateDirections.add(goalSector);
                }
            } else {
                candidateDirections.add((leftBorderSector + rightBorderSector) / 2);
            }
        }
        return candidateDirections;
    }

    private ArrayList<CandidateValley> getCandidateValleys(Map<Integer, Integer> binaryHistogram) {
        Object[] objectKeys = binaryHistogram.keySet().toArray();
        List<Integer> sortedKeys = Arrays.asList(Arrays.copyOf(objectKeys, objectKeys.length, Integer[].class));
        Collections.sort(sortedKeys);
        ArrayList<Integer> keys = new ArrayList<Integer>(sortedKeys);
        ArrayList<CandidateValley> candidateValleys = new ArrayList<CandidateValley>();

        int index = 0;
        Integer startSector = null;
        while (index < keys.size()) {
            if (binaryHistogram.get(keys.get(index)) == 0) {
                if(startSector == null) {
                    startSector = keys.get(index);
                }
            } else {
                if(startSector != null) {

                    candidateValleys.add(new CandidateValley(keys.get(index - 1), startSector));
                    startSector = null;
                }
            }
            index++;
        }
        if (startSector != null) {

            candidateValleys.add(new CandidateValley(keys.get(keys.size() - 1), startSector));
        }
        return candidateValleys;
    }

    public Map<Integer, Integer> buildBinaryHistogram(Map<Integer, Double> histogram) {
        Object[] objectKeys = histogram.keySet().toArray();
        List<Integer> sortedKeys = Arrays.asList(Arrays.copyOf(objectKeys, objectKeys.length, Integer[].class));
        Collections.sort(sortedKeys);
        ArrayList<Integer> keys = new ArrayList<Integer>(sortedKeys);

        Map<Integer, Integer> binaryHistogram = new HashMap<Integer, Integer>();
        boolean noObstacles = true;
        for (Integer key : keys) {
            double magnitude = histogram.get(key);
            if(magnitude > thresholdHigh) {
                binaryHistogram.put(key, 1);
                noObstacles = false;
            } else if(magnitude < thresholdLow) {
                binaryHistogram.put(key, 0);
            } else {
                Integer previousMagnitude = binaryHistogram.get(key - 1);
                binaryHistogram.put(key, (previousMagnitude == null) ? 0 : previousMagnitude);
            }
        }
        if (noObstacles) {
            return null;
        }
        return binaryHistogram;
    }


    public  Map<Integer, Double> buildPolarHistogram(double distanceToCarrotPoint, double distances[]) {

        double d_max = distanceToCarrotPoint;
        Map<Integer, Double> sector_magnitudes = new HashMap<Integer, Double>();
        double b = a / d_max;
        double enlargement_angle;
        double direction_to_obstacle;

        int maxSector = NO_OF_SECTORS /2;
        if(NO_OF_SECTORS % 2 == 0) {
           maxSector = maxSector - 1;
        }
        for(int i = -NO_OF_SECTORS/2; i <= maxSector; i++) {
            sector_magnitudes.put(i, 0.0);
        }

        for(int i = 0; i < distances.length; i++) {
            if(distances[i] <= d_max) {
                double magnitude = a - (b*distances[i]);

                enlargement_angle = Math.asin(ENLARGEMENT_RADIUS / distances[i]);
                direction_to_obstacle = startAngle + (i*angleIncrement);

                int startSector = Math.max((int)((direction_to_obstacle - enlargement_angle) / angularResolution), - (NO_OF_SECTORS / 2));

                int endSector =  Math.min((int) ((direction_to_obstacle + enlargement_angle) / angularResolution) + 1, maxSector);

                for(int k = startSector; k <= endSector; k++) {
                    if (!sector_magnitudes.containsKey(k) || (sector_magnitudes.get(k) < magnitude)) {
                        sector_magnitudes.put(k, magnitude);
                    }
                }
            }
        }

        return sector_magnitudes;
    }

    private class CandidateValley {

        private int leftBorderSector;
        private int rightBorderSector;

        public CandidateValley(int leftBorderSector, int rightBorderSector) {
            this.leftBorderSector = leftBorderSector;
            this.rightBorderSector = rightBorderSector;
        }

        public int getRightBorderSector () {
            return rightBorderSector;
        }

        public int getLeftBorderSector() {
            return leftBorderSector;
        }

        public boolean isWideValley(int sMAX) {
            return (leftBorderSector - rightBorderSector) > sMAX;
        }
    }
}