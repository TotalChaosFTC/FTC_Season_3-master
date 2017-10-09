package org.firstinspires.ftc.teamcode.Autonomous;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.teamcode.navx.ftc.AHRS;
import org.firstinspires.ftc.teamcode.navx.ftc.navXPIDController;

import java.util.Vector;

/**
 * TeleOp Mode
 * <p>
 *Enables control of the robot via the gamepad
 */
public abstract class NewBaseAutoOp extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor armTwist;
    SensorMRRangeSensor rangeSensor;
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();
    //navx values
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    private final double TARGET_ANGLE_DEGREES = 90.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private final int DEVICE_TIMEOUT_MS = 500;
    ColorSensor color1;
    Vector<Step> steps;
    Step currentStep;
    int counts = 0;
    int currentStepIndex;
    final static int ATREST = 0;
    final static int WAITFORRESETENCODERS = 1;
    final static int WAITFORCOUNTS = 2;
    final static int FINISHED = 3;
    final static int WAITFORCOLOR = 4;
    final static int WAITFORTURN = 5;
    int state = ATREST;
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 1;
    final static double WHEEL_DIAMETER = 2.75;
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    final static int MOVE = 1;
    final static int RIGHT = 2;
    final static int LEFT = 3;
    final static int RANGE = 4;
    final static int WAITFORTOUCH = 5;
    final static int BACK = 6;
    final static int BLUE = 7;
    final static int RED = 8;
    final static int WAIT = 9;
    final static int MOVEPUSHER = 10;
    final static int FORWARD = 11;
    final static int BACKWARD = 12;
    final static int NONE = 13;

    public class Step {
        public double distance;
        public double leftPower;
        public double rightPower;
        public int rightCounts;
        public int leftCounts;
        public int sweeperDirection;
        public double armPosition;
        public int sType;
        public Step(double dist, double left, double right, int stepType, int direction) {
            distance = dist;
            sType = stepType;
            initializeNavX();
            navx_device.zeroYaw();
            if (stepType == MOVE){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = left;
                rightPower = right;
            }
            else if (stepType == RIGHT) {
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = right;

            }
            else if(stepType == LEFT){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = left;
                rightPower = -right;
            }
            else if(stepType == BACK){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = -right;
            }
            else if(stepType == BLUE){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = -right;
            }
            else if(stepType == RED){
                rightCounts = convertDistance(distance);
                leftCounts = rightCounts;
                leftPower = -left;
                rightPower = -right;
            }
            else if(stepType == WAIT){
                leftPower = 0;
                rightPower = 0;
            }
            else{
                armPosition = dist;
            }
            sweeperDirection = direction;
        }

        public int convertDistance(double distance){
            double  rotations = distance / CIRCUMFERENCE;
            double counts = ENCODER_CPR * rotations * GEAR_RATIO;
            return (int) counts;
        }
    }

    public void initializeNavX(){

        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);

    }
    public void init()
    {
        leftFront = hardwareMap.dcMotor.get("lf");
        rightFront = hardwareMap.dcMotor.get("rf");
        leftBack = hardwareMap.dcMotor.get("lb");
        rightBack = hardwareMap.dcMotor.get("rb");
        rangeSensor = hardwareMap.get(SensorMRRangeSensor.class, "rgn");
        navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("navx"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
        steps = new Vector<Step>();
        initSteps();
        currentStep = steps.get(0);
        currentStepIndex = 0;
    }
// Create all doubles for sensors ect..

    public abstract void initSteps();
    @Override
    public void loop(){
        if (state == ATREST){
            if(currentStep.sType == WAIT){
                if (counts == 400){
                    currentStepIndex = currentStepIndex + 1;
                    counts = 0;
                    if (currentStepIndex >= steps.size()) {
                        state = FINISHED;
                    } else {
                        currentStep = steps.get(currentStepIndex);
                        state = ATREST;

                    }
                }
                else{
                    counts = counts+1;                }
            }
            else {
                resetEncoders();
                state = WAITFORRESETENCODERS;
            }
        }
        else if( state == WAITFORRESETENCODERS) {
            if (areEncodersReset()){
                setMotorPower(currentStep.leftPower, currentStep.rightPower);
                state = WAITFORCOUNTS;
            }
        }
        else if (state== WAITFORCOUNTS) {
            if (areCountsReached(currentStep.leftCounts, currentStep.rightCounts)) {
                setMotorPower(0, 0);
                currentStepIndex = currentStepIndex + 1;
                if (currentStepIndex >= steps.size()) {
                    state = FINISHED;
                } else {
                    currentStep = steps.get(currentStepIndex);
                    state = ATREST;
                }
            }
            else {
                setMotorPower(currentStep.leftPower, currentStep.rightPower);
                navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
                try {
                    if ( yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS ) ) {
                        if ( yawPIDResult.isOnTarget() ) {
                            setMotorPower(currentStep.leftPower, currentStep.rightPower);

                        } else {
                            double output = yawPIDResult.getOutput();
                            if ( output < 0 ) {
                                setMotorPower(currentStep.leftPower - output, currentStep.rightPower + output);
                            } else {
                                setMotorPower(currentStep.leftPower + output, currentStep.rightPower - output);

                            }
                        }
                    } else {
                        Log.w("navXRotateToAnglePIDOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                    }
                }
                catch(InterruptedException e){

                }

            }
        }
        else if (state == FINISHED){
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    }

    @Override
    public void stop(){
        state = ATREST;
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public boolean areEncodersReset() {
        return leftFront.getCurrentPosition() == 0 &&
                rightFront.getCurrentPosition() == 0 &&
                leftBack.getCurrentPosition() == 0 &&
                rightBack.getCurrentPosition() == 0;
    }

    public boolean areCountsReached(int leftCounts, int rightCounts) {
        return ( Math.abs(leftFront.getCurrentPosition()) >= Math.abs(leftCounts) &&
                Math.abs(rightFront.getCurrentPosition()) >= Math.abs(rightCounts) &&
                Math.abs(leftBack.getCurrentPosition())>= Math.abs(leftCounts) &&
                Math.abs(rightBack.getCurrentPosition()) >= Math.abs(rightCounts) );

    }

    public void setMotorPower(double leftPower,double rightPower){
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setPower(leftPower);
        rightFront.setPower(rightPower);
        leftBack.setPower(leftPower);
        rightBack.setPower(rightPower);
    }
}