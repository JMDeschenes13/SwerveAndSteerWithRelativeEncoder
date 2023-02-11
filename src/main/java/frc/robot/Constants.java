// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {

    class DriveConstants{
        //DriveMotors
        public static final int kFrontLeftDrivePort = 4;
        public static final int kFrontRightDrivePort = 3;
        public static final int kBackLeftDrivePort = 2;
        public static final int kBackRightDrivePort = 1;

        //Truning Motors
        public static final int kFrontLeftTurningPort = 8;
        public static final int kFrontRightTuringPort = 7;
        public static final int kBackLeftTurningPort = 6;
        public static final int kBackRightTurningPort = 5;

        //TurningEncoderChannels
        public static final int kFrontLeftEncoderChannelA = 6;
        public static final int kFrontLeftEncoderChannelB = 7;
        public static final int kFrontRightEncoderChannelA = 4;
        public static final int kFrontRightEncoderChannelB = 5;
        public static final int kBackLeftEncoderChannelA = 2;
        public static final int kBackLeftEncoderChannelB = 3;
        public static final int kBackRightEncoderChannelA = 0;
        public static final int kBackRightEncoderChannelB = 1;

        //Encoders inverted
        public static final boolean kFrontLeftEncoderInverted = false;
        public static final boolean kFrontRightEncoderInverted = false;
        public static final boolean kBackLeftEncoderInverted = false;
        public static final boolean kBackRightEncoderInverted = false;

        //Drive Motors Inverted
        public static final boolean kfrontLeftDriveMotorInverted = true;
        public static final boolean kfrontRightDriveMotorInverted = false;
        public static final boolean kbackLeftDriveMotorInverted = true;
        public static final boolean kbackRightDriveMotorInverted = false;

        //Turning Motors Inverted
        public static final boolean kfrontLeftTurningMotorInverted = false;
        public static final boolean kfrontRightTurningMotorInverted = false;
        public static final boolean kbackLeftTurningMotorInverted = false;
        public static final boolean kbackRightTurningMotorInverted = false;

    }
}
