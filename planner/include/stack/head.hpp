#ifndef HEAD_HPP
#define HEAD_HPP
   struct Coordinates
    {
        double latitude;
        double longitude;
    };

    enum status{
        goalReached,
        goalNext,
        goalNow
    };

    enum NavStatus{
        Manual,
        GoalNotYetProvided,
        Wait,
        GoalSeeking,
        GnssError
    
    };


    enum class elog{
        init,
        goalInit,
        gpsInit,
        pclInit,
        imuInit,
        gpsActive,
        pclActive,
        imuActive,
        manual,
        currentGoal  
        //coolAsh
    };

#endif
