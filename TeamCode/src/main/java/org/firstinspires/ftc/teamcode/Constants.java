package org.firstinspires.ftc.teamcode;

public class Constants {
    public static class Knob{
        public static double TICKS = 384.5;
    }
    public static class Friction{
        public static double LEFT_BOUND = -1 * Knob.TICKS;
        public static double RIGHT_BOUND = 1 * Knob.TICKS;
    }
    public static class FRICTIONLESS{
        public static double DRIVE_POWER = 0.1;
    }
    public static class ROLLOVER{
        public static int SECTIONS = 6;
        public static double RESISTANCE = 0.1;
    }
}
