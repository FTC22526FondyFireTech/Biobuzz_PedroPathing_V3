package org.firstinspires.ftc.teamcode.utils;
public class RollingAverage {
    private final double[] buffer;
    private int head;
    private int count;
    private double sum;

    public RollingAverage(int windowSize) {
        this.buffer = new double[windowSize];
        this.head = 0;
        this.count = 0;
        this.sum = 0.0;
    }

    public void add(double value) {
        sum -= buffer[head];
        buffer[head] = value;
        sum += value;
        head = (head + 1) % buffer.length;
        if (count < buffer.length) {
            count++;
        }
    }

    public double getAverage() {
        return count == 0 ? Double.NaN : sum / count;
    }

    public static void main(String[] args) {
        RollingAverage rollingAverage = new RollingAverage(20);
        double[] data = {1, 3, 5, 6, 8, 12, 18, 21, 22, 25, 28, 30, 32, 35, 37, 40, 42, 45, 47, 50};

        for (double value : data) {
            rollingAverage.add(value);
           // System.out.println("New number added is " + value + ", Rolling Average = " + rollingAverage.getAverage());
        }
    }
}