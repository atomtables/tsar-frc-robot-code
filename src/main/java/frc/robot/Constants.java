package frc.robot;

//Drive Constants need to be updated since we have krakens and use swerve

public final class Constants {
    public static final class DriveConstants {
      public static final int LEFT_LEADER_ID = 1;
      public static final int LEFT_FOLLOWER_ID = 2;
      public static final int RIGHT_LEADER_ID = 3;
      public static final int RIGHT_FOLLOWER_ID = 4;
  
      public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
      public static final double DRIVE_MOTOR_VOLTAGE_COMP = 12;
      public static final double SLOW_MODE_MOVE = 0.5;
      public static final double SLOW_MODE_TURN = 0.6;
    }
  
    public static final class RollerConstants {
      public static final int ROLLER_MOTOR_ID = 5;
      public static final int ROLLER_MOTOR_CURRENT_LIMIT = 60;
      public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
      public static final double ROLLER_CORAL_OUT = -.4;
      public static final double ROLLER_ALGAE_IN = -0.8;
      public static final double ROLLER_ALGAE_OUT = 0.4;
      public static final double ROLLER_CORAL_STACK = -1;
    }
  
    public static final class ArmConstants {
      public static final int ARM_MOTOR_ID = 6;
      public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
      public static final double ARM_MOTOR_VOLTAGE_COMP = 10;
      public static final double ARM_SPEED_DOWN = 0.4;
      public static final double ARM_SPEED_UP = -0.4;
      public static final double ARM_HOLD_DOWN = 0.1;
      public static final double ARM_HOLD_UP = -0.15;
    }
  
    public static final class ClimberConstants {
      public static final int CLIMBER_MOTOR_ID = 7;
      public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 60;
      public static final double CLIMBER_MOTOR_VOLTAGE_COMP = 12;
      public static final double CLIMBER_SPEED_DOWN = -0.5;
      public static final double CLIMBER_SPEED_UP = 0.5;
    }
  
    public static final class OperatorConstants {
      public static final int DRIVER_CONTROLLER_PORT = 0;
      public static final int OPERATOR_CONTROLLER_PORT = 1;
    }
}