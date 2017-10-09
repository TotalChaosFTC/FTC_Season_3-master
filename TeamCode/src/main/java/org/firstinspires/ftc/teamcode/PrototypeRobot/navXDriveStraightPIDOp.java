package org.firstinspires.ftc.teamcode.PrototypeRobot;

import org.firstinspires.ftc.teamcode.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBase;

/*
 * An example linear op mode where the robot will drive in
 * a straight line (where the driving directon is guided by
 * the Yaw angle from a navX-Model device).
 *
 * This example uses a simple PID controller configuration
 * with a P coefficient, and will likely need tuning in order
 * to achieve optimal performance.
 *
 * Note that for the best accuracy, a reasonably high update rate
 * for the navX-Model sensor should be used.  This example uses
 * the default update rate (50Hz), which may be lowered in order
 * to reduce the frequency of the updates to the drive system.
 */
@Autonomous(name="StraightLineDrive", group="SDV")
public class navXDriveStraightPIDOp extends AutoBase {
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;

    /* This is the port on the Core Device Interace Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */
    //private final int NAVX_DIM_I2C_PORT = 0;
    //private AHRS navx_device;
    //private ElapsedTime runtime = new ElapsedTime();

    //navx values
    //private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 90.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int DEVICE_TIMEOUT_MS = 500;


    //encoder values
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);




    @Override
    public void runOpMode() throws InterruptedException {
        int counter = 0;
        boolean nullFlag = true;
        while (counter < 5 && nullFlag) {
            try {
                frontRight = hardwareMap.get(DcMotor.class, "rf");
                frontLeft = hardwareMap.get(DcMotor.class, "lf");
                backRight = hardwareMap.get(DcMotor.class, "rb");
                backLeft = hardwareMap.get(DcMotor.class, "lb");

                /*navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("navx"),
                        NAVX_DIM_I2C_PORT,
                        AHRS.DeviceDataType.kProcessedData,
                        NAVX_DEVICE_UPDATE_RATE_HZ);
                        */
                nullFlag = false;
            } catch (NullPointerException e) {
                counter++;
            }
        }




        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();
        runUsingEncoders();



        waitForStart();


        //initializeNavX();
        final double TOTAL_RUN_TIME_SECONDS = 10.0;

        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        encoderDrive(0.5, 15);//,yawPIDResult);
        //encoderTurn(90,0.5, yawPIDResult);
    }
}