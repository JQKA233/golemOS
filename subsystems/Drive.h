# include "ctre/Phoenix.h"


enum DriveControlState {
    FORCE_ORIENT,
    OPEN_LOOP,
    HEADING_CONTROL,
    VELOCITY,
    PATH_FOLLOWING,
    AUTO_BALANCE
};

class Drive
{
private:
    

public:
    Drive();
    ~Drive();
};

Drive::Drive()
{
}

Drive::~Drive()
{
}
