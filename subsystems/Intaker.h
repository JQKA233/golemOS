#pragma once

#include "ctre/Phoenix.h"
#include <frc/motorcontrol/PWMTalonFX.h>


class Intaker
{
private:
    frc::PWMTalonFX collect_moto{8};
public:
    Intaker()
    {
    }

    ~Intaker()
    {
    }
    
    
};



