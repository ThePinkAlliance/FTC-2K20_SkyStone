package org.firstinspires.ftc.PinkCode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

// Abstract Class to Define the Controls of the Gamepads
public abstract class Controls extends OpMode {
    // Define Variables
    private static boolean base_right_bumper_pressed = false;
    private static boolean base_left_bumper_pressed = false;
    private static boolean base_b_pressed = false;
    private static boolean base_y_pressed = false;
    private static boolean tower_right_bumper_pressed = false;
    private static boolean tower_left_bumper_pressed = false;
    private static boolean tower_dpad_right_pressed = false;
    private static boolean tower_dpad_left_pressed = false;
    private static boolean tower_start_pressed = false;
    private static boolean tower_back_pressed = false;
    private static boolean tower_a_pressed = false;
    private static boolean tower_b_pressed = false;
    private static boolean tower_y_pressed = false;

    // Base Gamepad
    public static boolean base_right_joystick_button;
    public static boolean base_left_joystick_button;
    public static float base_right_joystick;
    public static float base_left_joystick;
    public static float base_right_trigger;
    public static float base_left_trigger;
    protected boolean base_right_bumper(boolean toggle) {
        if (toggle) {
            if (!base_right_bumper_pressed && gamepad1.b) {
                base_right_bumper_pressed = true;
                return true;
            } else {
                base_right_bumper_pressed = false;
                return false;
            }
        } else {
            return gamepad1.right_bumper;
        }
    }
    public boolean base_left_bumper(boolean toggle) {
        if (toggle) {
            if (!base_left_bumper_pressed && gamepad1.left_bumper) {
                base_left_bumper_pressed = true;
                return true;
            } else {
                base_left_bumper_pressed = false;
                return false;
            }
        } else {
            return gamepad1.left_bumper;
        }
    }
    public static boolean base_dpad_right;
    public static boolean base_dpad_left;
    public static boolean base_dpad_down;
    public static boolean base_dpad_up;
    public static boolean base_start;
    public static boolean base_back;
    public static boolean base_a;
    protected boolean base_b(boolean toggle) {
        if (toggle) {
            if (!base_b_pressed && gamepad1.b) {
                base_b_pressed = true;
                return true;
            } else {
                base_b_pressed = false;
                return false;
            }
        } else {
            return gamepad1.b;
        }
    }
    public static boolean base_x;
    protected boolean base_y(boolean toggle) {
        if (toggle) {
            if (!base_y_pressed && gamepad1.y) {
                base_y_pressed = true;
                return true;
            } else {
                base_y_pressed = false;
                return false;
            }
        } else {
            return gamepad1.y;
        }
    }

    // Tower Gamepad
    public static boolean tower_right_joystick_button;
    public static boolean tower_left_joystick_button;
    public static float tower_right_joystick;
    public static float tower_left_joystick;
    public static float tower_right_trigger;
    public static float tower_left_trigger;
    public boolean tower_right_bumper(boolean toggle) {
        if (toggle) {
            if (!tower_right_bumper_pressed && gamepad2.right_bumper) {
                tower_right_bumper_pressed = true;
                return true;
            } else {
                tower_right_bumper_pressed = false;
                return false;
            }
        } else {
            return gamepad2.right_bumper;
        }
    }
    public boolean tower_left_bumper(boolean toggle) {
        if (toggle) {
            if (!tower_left_bumper_pressed && gamepad2.left_bumper) {
                tower_left_bumper_pressed = true;
                return true;
            } else {
                tower_left_bumper_pressed = false;
                return false;
            }
        } else {
            return gamepad2.left_bumper;
        }
    }
    public boolean tower_dpad_right(boolean toggle) {
        if (toggle) {
            if (!tower_dpad_right_pressed && gamepad2.dpad_right) {
                tower_dpad_right_pressed = true;
                return true;
            } else {
                tower_dpad_right_pressed = false;
                return false;
            }
        } else {
            return gamepad2.dpad_right;
        }
    }
    public boolean tower_dpad_left(boolean toggle) {
        if (toggle) {
            if (!tower_dpad_left_pressed && gamepad2.dpad_left) {
                tower_dpad_left_pressed = true;
                return true;
            } else {
                tower_dpad_left_pressed = false;
                return false;
            }
        } else {
            return gamepad2.dpad_left;
        }
    }
    public boolean tower_dpad_down;
    public boolean tower_dpad_up;
    public boolean tower_start(boolean toggle) {
        if (toggle) {
            if (!tower_start_pressed && gamepad2.start) {
                tower_start_pressed = true;
                return true;
            } else {
                tower_start_pressed = false;
                return false;
            }
        } else {
            return gamepad2.start;
        }
    }
    public boolean tower_back(boolean toggle) {
        if (toggle) {
            if (!tower_back_pressed && gamepad2.back) {
                tower_back_pressed = true;
                return true;
            } else {
                tower_back_pressed = false;
                return false;
            }
        } else {
            return gamepad2.back;
        }
    }
    public boolean tower_a(boolean toggle) {
        if (toggle) {
            if (!tower_a_pressed && gamepad2.a) {
                tower_a_pressed = true;
                return true;
            } else {
                tower_a_pressed = false;
                return false;
            }
        } else {
            return gamepad2.a;
        }
    }
    public boolean tower_b(boolean toggle) {
        if (toggle) {
            if (!tower_b_pressed && gamepad2.b) {
                tower_b_pressed = true;
                return true;
            } else {
                tower_b_pressed = false;
                return false;
            }
        } else {
            return gamepad2.b;
        }
    }
    public boolean tower_x;
    public boolean tower_y(boolean toggle) {
        if (toggle) {
            if (!tower_y_pressed && gamepad2.y) {
                tower_y_pressed = true;
                return true;
            } else {
                tower_y_pressed = false;
                return false;
            }
        } else {
            return gamepad2.y;
        }
    }
}