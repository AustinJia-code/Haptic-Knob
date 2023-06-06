package org.firstinspires.ftc.teamcode;

public class Constants {
    public static class Knob{
        /** Ticks increase clockwise */
        public static double TICKS = 384.5;
    }
    public static class Friction{
        public static double RANGE_DEGREES = 180;
        public static double RANGE_TICKS = RANGE_DEGREES / 360.0 * Knob.TICKS;
        public static int LEFT_BOUND_TICKS = (int) (-1 * RANGE_TICKS / 2.0);
        public static int RIGHT_BOUND_TICKS = (int) (1 * RANGE_TICKS / 2.0);
        public static double DEADZONE_DEGREES = 2.0;
        public static double DEADZONE_SCALED = DEADZONE_DEGREES / RANGE_DEGREES;
        public static double K_P = 1 / DEADZONE_SCALED;
    }
    public static class Frictionless {
        public static double RANGE_DEGREES = 180;
        public static double RANGE_TICKS = RANGE_DEGREES / 360.0 * Knob.TICKS;
        public static double DRIVE_POWER = 0.1;
        public static double K_P = 0.2;
        public static double MAX_ACCELERATION = Knob.TICKS / 2.0;
    }
    public static class Detent{
        public static int SECTIONS = 6;
        public static int SECTION_RANGE_DEGREES = 360 / SECTIONS;
        public static int SECTION_RANGE_TICKS = (int) (Knob.TICKS / SECTIONS);
        public static double K_P = 0.1;
    }
    public static class Output {
        public static double RANGE_DEGREES = 180;
        public static double RANGE_TICKS = RANGE_DEGREES / 360 * Knob.TICKS;
        public static int LEFT_BOUND_TICKS = (int) (-1 * RANGE_TICKS / 2.0);
        public static int RIGHT_BOUND_TICKS = (int) (1 * RANGE_TICKS / 2.0);

        public static double K_P = 0.1;
    }
}
