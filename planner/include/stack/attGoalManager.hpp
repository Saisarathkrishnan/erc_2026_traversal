#include<iostream>
#include "head.hpp"

class goal{
    public:
        goal(){
            std::cout<<"init"<<std::endl;
            reached=false;
        }
        goal(Coordinates destination, Coordinates startPt, Coordinates currentPt):
        destLocal(destination),
        destGNSSG{0,0},
        startGNSSG(startGNSSG),
        curr(currentPt),
        reached(false)
        {
            destGNSSG=changeToGlobal(destination);
            
            std::cout<<"init elite"<<std::endl;
        }
        void setGoal(Coordinates destination, Coordinates start){
            destGNSSG=destination;
            startGNSSG=start;
        }

        Coordinates changeToGlobal(Coordinates local){
            Coordinates global;
            global= local;
            return global; //change
        }
        
        void update(Coordinates currentPt){            
            curr=currentPt;
        }

        bool GoalAchieved(){
            return reached;
        }

        void GoalReached(){
            reached=true;
        }
        
        ~goal(){
            std::cout<<"delete all goal class :)"<<std::endl;
        }

        Coordinates GoalGNSS(){
            return destGNSSG;
        }

    private:
        Coordinates destLocal;
        Coordinates destGNSSG;
        Coordinates startGNSSG;
        Coordinates curr;
        bool reached;



};