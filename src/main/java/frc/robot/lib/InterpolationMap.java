// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import java.util.Map;

/** Add your docs here. */
public class InterpolationMap {
    public static class MapPoint {
        public double in;
        public double out;

        public MapPoint(double in, double out) {
            this.in = in;
            this.out = out;
        }
    };

    private MapPoint[] mapPoints;

    public InterpolationMap(MapPoint[] mapPoints) {
        this.mapPoints = mapPoints;
    }

    public double interpolate(double in) {
        if (in < mapPoints[0].in) {
            return mapPoints[0].out;
        }

        int last = mapPoints.length - 1;
        if (in > mapPoints[last].in) {
            return mapPoints[last].out;
        }

        for (int index = 1; index < mapPoints.length; index++) {
            MapPoint floor = mapPoints[index - 1];
            MapPoint ceiling = mapPoints[index];

            if (floor.in <= in && in <= ceiling.in) {
                return interpolate(in, floor.in, floor.out, ceiling.in, ceiling.out);
            }
        }

        return mapPoints[0].out;
    }

    private double interpolate(double in, double floorIn, double floorOut, double ceilingIn, double ceilingOut) {
        return (floorOut * (ceilingIn - in)
                + ceilingOut * (in - floorIn))
                / (ceilingIn - floorIn);
    }
}
