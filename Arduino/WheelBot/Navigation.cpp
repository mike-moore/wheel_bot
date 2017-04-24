#include "Navigation.h"

void Navigation::InitSensors() {

}

void Navigation::Execute() {


    /// - TODO Get data from sensors here. Perform any filtering prior
    ///   to sending it on for Guidance.
    State.SensedHeading = 1.0;
    State.SensedDistance = 2.0;

}

