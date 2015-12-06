package com.qualcomm.ftcrobotcontroller.opmodes.customcode;

import com.qualcomm.ftcrobotcontroller.opmodes.customcode.RobotDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Tutorial extends OpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private Servo leftGripper;
    private Servo rightGripper;
    private OpticalDistanceSensor opticalDistanceSensor;
    private ElapsedTime timer;
    private double lightValue = .03;
    private double darkValue = .01;
    private double threshold = (lightValue + darkValue) / 2;
    private RobotDrive myRobotDrive;

    enum State {Drive, Backup, Turn}
    State state;
    final double BACKUP_TIME = 0.8;
    final double TURN_TIME = 0.7;

    public float leftY = -gamepad1.left_stick_y;
    public float rightY = -gamepad1.right_stick_y;
    public final double LEFT_OPEN_POSITION = 0.0;
    public final double LEFT_CLOSED_POSITION = 0.5;
    public final double RIGHT_OPEN_POSITION = 1.0;
    public final double RIGHT_CLOSED_POSITION = 0.5;



    @Override
    public void init() {
        myRobotDrive = new RobotDrive(hardwareMap.dcMotor.get("left_drive"), hardwareMap.dcMotor.get("right_drive"));
        //links the instance variables to the hardware
        leftMotor = hardwareMap.dcMotor.get("left_drive");
        rightMotor = hardwareMap.dcMotor.get("right_drive");

        leftGripper = hardwareMap.servo.get("left_hand");
        rightGripper = hardwareMap.servo.get("right_hand");

        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("sensor_EOPD");
        //reverses the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        state = state.Drive;

        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        if (leftY > 0.0) {
           leftMotor.setPower(leftY);
        }
        if (rightY > 0.0) {
            rightMotor.setPower(rightY);

            if (gamepad1.x) {
                leftGripper.setPosition(LEFT_OPEN_POSITION);
                rightGripper.setPosition(RIGHT_OPEN_POSITION);
            }
            if (gamepad1.y) {
                leftGripper.setPosition(LEFT_CLOSED_POSITION);
                rightGripper.setPosition(RIGHT_CLOSED_POSITION);
            }
        }

        double reflectance = opticalDistanceSensor.getLightDetected();

        switch(state) {
            case Drive:
                if (reflectance < threshold) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    state = state.Backup;
                    timer.reset();
                } else {
                    leftMotor.setPower(0.15);
                    rightMotor.setPower(0.15);
                }
                break;
            case Backup:
                leftMotor.setPower(-0.25);
                rightMotor.setPower(-0.25);

                if (timer.time() >= BACKUP_TIME) {
                    state = state.Turn;
                    timer.reset();
                }
                break;
            case Turn:
                leftMotor.setPower(0.25);
                rightMotor.setPower(-0.25);

                if (timer.time() >= TURN_TIME) {
                    state = state.Drive;
                }
                break;
        }

        telemetry.addData("Current State", state.name());
        telemetry.addData("Reflective Value", reflectance);
    }
}
