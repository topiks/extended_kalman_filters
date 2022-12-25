#ifndef DATA_DUMMY_H
#define DATA_DUMMY_H

class DataDummy
{
    
public:
    int get_acceleration(int t);
    int get_velocity(int t);
    int get_position(int t);

    int a, v, d;
    
};

int DataDummy::get_acceleration(int t)
{
    if(t <= 15)
        a = 0;
    else if(t > 15 && t <= 20)
        a = (8/5)*(t-15);
    else if(t > 20)
        a = 8;

    return a;
}

int DataDummy::get_velocity(int t)
{
    if(t <= 15)
        v = 50;
    else if(t > 15)
        v = (325/35)*(t-15) + 50;

    return v;
}

int DataDummy::get_position(int t)
{
    d = 3200 + get_velocity(t)*t + 0.5*get_acceleration(t)*t*t;

    return d;
}


#endif