package com.qualcomm.ftcrobotcontroller.opmodes.customcode;


import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotDrive {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public RobotDrive(DcMotor leftMotor, DcMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void tankDrive(double leftValue, double rightValue) {
        leftMotor.setPower(leftValue);
        rightMotor.setPower(rightValue);
    }

    public void tankDrive(double leftValue, double rightValue, boolean squareInputs) {
        if(squareInputs) {
            if(leftValue >= 0.0) {
                leftValue = (leftValue * leftValue);
            } else {
                leftValue = -(leftValue * leftValue);
            }
        }
        if(rightValue >= 0.0) {
            rightValue = (rightValue * rightValue);
        } else {
            rightValue = -(rightValue * rightValue);
        }

        tankDrive(leftValue, rightValue);
    }
}
