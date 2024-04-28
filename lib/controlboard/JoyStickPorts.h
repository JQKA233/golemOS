#pragma once

class JoyStickPorts
{
    private:
        /* data */
    public:
        enum Button{
            NONE,
            A,
            B,
            X,
            Y,
            LB,
            RB,
            TWO_WINDOWS,
            THREE_LINES
        };

        enum Axis{
            LX,
            LY,
            LA,
            RA,
            RX,
            RY
        };
        enum Direction
        {
            FORWARD,
            BACKWARD,
            LEFT,
            RIGHT,
            FORWARD_LEFT,
            FORWARD_RIGHT,
            BACKWARD_LEFT,
            BACKWARD_RIGHT
        };
        JoyStickPorts(){
        }
        ~JoyStickPorts(){
        }
};
