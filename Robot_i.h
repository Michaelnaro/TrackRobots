#ifndef ROBOT_I_H_
#define ROBOT_I_H_

class Robot_i
{
public:
	Robot_i();
	~Robot_i();

protected:
	int id;
	unsigned long timestamp;  /* time in miliseconds */
	double theta;
	double acceleration;
};



#endif /* ROBOT_I_H_ */