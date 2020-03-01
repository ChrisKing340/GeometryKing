#include "Acceleration.h"

namespace King {

    /******************************************************************************
    *    Method_Name
    *        Desc:       
    *        Input:      
    *        Output:     
    *        Remarks:    
    ******************************************************************************/

    Acceleration operator/(const Force & f, const UnitOfMeasure::Mass & m)
    {
        return Acceleration(static_cast<float>(f.Get_magnitude()) / static_cast<float>(m), f.Get_unit_direction());
    }

}