package org.firstinspires.ftc.teamcode.util;

public final class Constants {
    //math constants to speed up calculations
    //public static final double SQRT2=Math.sqrt(2);
    public static final double PI_OVER4=Math.PI/4;
    //public static final double PI_OVER2=Math.PI/2;
    //public static final double INCHES_PER_METER=39.3701;
    //public static final double PI_OVER_180=Math.PI/180;

    //public static final double UPPER_BELT_POWER = 0.8;
    // ARM release and set position
    //public static final double ARM_RELEASE_POS = 0.85;
    //public static final double ARM_SET_POS = 0.2;

    public static final double Y_DISTANCE_RATIO = 12.0/13.0;
    public static final double X_DISTANCE_RATIO = 30.0/26.0;

    // motor constants based on physical properties of the robot
    public static final double COUNTS_PER_MOTOR_REV = 1120;   // 1120 per revolution
    public static final double DRIVE_GEAR_REDUCTION = 0.75; //   3/4
    public static final double WHEEL_DIAMETER_INCHES = 96/25.4;     // For figuring circumference
    public static final double COUNTS_PER_INCH_FORWARD = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    //public static final double BELT_COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    //public static final double INCH_PER_COUNTS_FORWARD = 1/COUNTS_PER_INCH_FORWARD;
    //public static final double INCH_PER_COUNT_TIMES_1_OVER_ROOT_2 = INCH_PER_COUNTS_FORWARD * 1/SQRT2;
    public static final double ROBOT_DIAMETER_IN = 13;
    public static final double COUNTS_PER_ROTATE = (ROBOT_DIAMETER_IN * Math.PI)*COUNTS_PER_INCH_FORWARD;
    public static final int [] FORWARD_VALUES  = new int[]{ 1, 1, 1, 1,1};
    public static final int [] REVERSE_VALUES = new int[]{-1, -1, -1, -1,-1};
    public static final int [] LATERAL_LEFT_VALUES = new int[]{1,-1,1,-1,1};
    public static final int [] LATERAL_RIGHT_VALUES = new int[]{-1,1,-1,1,1};
    public static final int [] ROTATE_VALUES = new int[]{1,1,-1,-1};

    public static final double SPEED_FACTOR =2.0;
    public static final double ROTATION_RATE =1.0;


    // speed settings
    public static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    //public static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    public static final double     HEADING_THRESHOLD       = 1 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value
    public static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    //maybe only use one of these.
    public static final double     P_DRIVE_GAIN           = 0.025;     // Larger is more responsive, but also less stable


    //public final static String VuforiaKey = "Ac6tBZr/////AAABmd2+0ZS1DUaxvjeLOQXt6BocTj8MS8ZdGc3iaWgJcb4x+GTRiMydjRed7kvoAvq0x21glktV2ekv6Nq8WLNelf5Chl5vN4X9QjUKYvH1fgh72q2cY2w5lMO5tmoOAbyNlN4hSM+RdaWXC7MpY95EVbwz584eP2KUQ97DMCFYqGj6zaVTap2FQ/U2rK7XDNp+s0mdm1+2dvJh6bw0Xpp/DjkUG7RB3uLZe0niObsnONPJg29RCf2eOVY/NP7qjXZamhGLjR1Cpj+U2HGh5DIqCauT/lvn/PDfa+H8ErXG0grgeSqQUHGYlsnYiYrp7Q70RKeebAeOsMVVj6zNhjI6dGE06u3JZgT6aF5EMxnJyc2X";

    // Arm control dead zone section
    public static final double armControlDeadzone = 0.2;

    public enum piecePositions {
        FRONT,
        REAR,
        CENTER
    }

    public enum pixelDropPositions {
        // center == 0.5 250ms
        // front == 0.85 250ms
        // back == 0.10 500ms
        RIGHT_CENTER(0.5,500, 0.0),
        RIGHT_FRONT(0.85,500, 0.0),
        RIGHT_REAR(0.10,500, 0.0),
        RIGHT_RESET(1.0,500, 1.0),
        LEFT_RESET(0.0,500,0.0),
        LEFT_CENTER(0.5,500, 1.0),
        LEFT_FRONT(0.10,500, 1.0),
        LEFT_REAR(0.85,500, 1.0);

        public final double pos;
        public final long ms;
        public final double flipPos;

        pixelDropPositions(double pos, long ms, double flipPos) {
            this.pos = pos;
            this.ms = ms;
            this.flipPos = flipPos;
        }
    }

    public enum autoSensorPositions {

        EXTENDED_POS(0.65,0.60),
        RETRACTED_POS(0.0,1.0);

        public final double frontPos;
        public final double rearPos;

        autoSensorPositions(double frontPos, double rearPos) {
            this.frontPos = frontPos;
            this.rearPos = rearPos;
        }
    }
    public static final int FRONT_SENSOR_SERVO=0, REAR_SENSOR_SERVO=1;

    public static final int FRONT_SENSOR_DISTANCE_THRESHOLD=30, REAR_SENSOR_DISTANCE_THRESHOLD=20;



    public static final class MecanumDrive {
        public static final double ZONE_LATERAL   = 0.2;
        public static final double ZONE_FORWARD   = 0.2;
        public static final double ZONE_ROTATION  = 0.1;
        public static final double RAMP_RATE_J1X  = 1.5;
        public static final double RAMP_RATE_J1Y  = 1.5;
        public static final double RAMP_RATE_J2X  =  1.5;
        //public static final double FINE_CONTROL   = 0.35;
    }
}
