/**
	This is a demo of an inverse kinematic algorithm

	Developed for Autonomous Robotics Club (ARC) at Purdue University
	
	This program is just a demo, and as of now is unfinished.
	All code utilized is public and may be distributed.


	@author Bryan M Yakimisky, ARC
	@version 0.1.1; September 18th, 2020



*/





#include <iostream>
#include <stdio.h> //print f
#include <math.h> //trig functions
#include <vector> //vector 

#define PI 3.14159265

using namespace std;

class Limb //The Limb class stores information regarding the position, rotation, and vector orientation of the limb.
{
	public:
		Limb(); //Default Constructor
		Limb(double); //Used for first Limb, initial position is set to (0,0)
		Limb(double, Limb); //Used for second Limb, initial position is calcualted using previously limb.
		double getXHead(); //Returns X Head Position Of Vector
		double getYHead(); // Returns Y Head Position of Vector
		double getYTail(); // Returns Y Tail position of vector
		double getXTail(); // Returns X Tail position of vector
		void updatePosition(double, double, double, double); //Updates both x and y values of head and tail
		void printCoords(); //Prints Head, Tail and Vector
	private:
		double xHead; //stores x head coordinate
		double yHead; //stores y head coordinate
		double xTail; //stores x tail coordinate
		double yTail; //stores y tail coordinate
		double rotation; //stores current rotation, set to 0 by default
		double length; //stores length

};
Limb::Limb()
{
}

Limb::Limb(double length) //Basic Lim Constructor, sets tail values to origin, and head x value to magnitude of length
{
	yTail = 0; 
	xTail = 0;
	xHead = length; 
	yHead = 0;
	rotation = 0;
	this->length = length;
}

Limb::Limb(double length, Limb limb) //Sets tail value to position of the head of previous vector, head is computer by adding length to tail x value.
{
	yTail = limb.getYHead();
	xTail = limb.getXHead();
	yHead = yTail;
	xHead = xTail + length;
	rotation = 0;
	this->length = length;
}

double Limb::getXHead() // returns x head coordinate
{
	return xHead;
}

double Limb::getYHead() // returns y head coordinate
{
	return yHead;
}

double Limb::getXTail() //returns x tail coordinate
{
	return xTail;
}

double Limb::getYTail() //returns y tail coordinate
{
	return yTail;
}

void Limb::updatePosition(double xHead, double yHead, double xTail, double yTail) //Updates position of vectors
{
	this->xHead = xHead;
	this->yHead = yHead;
	this->xTail = xTail;
	this->yTail = yTail;
}

void Limb::printCoords()
{
	printf("Tail:(%.2f,%.2f)\tHead:(%.2f,%.2f)\tVector:<%.2f,%.2f>\n",xTail,yTail,xHead,yHead,(xHead-xTail),(yHead-yTail)); //Prints Tail, Head and Vector Values
}

vector<Limb> rotateArm(vector<Limb>, int, double);


int main()
{
	int numLimbs; //number of independent joints
	
	cout << "How many limbs do you have: ";
	cin >> numLimbs;

	vector<Limb> arm(numLimbs); //Stores values of limbs in arm

	for (int i = 0; i < numLimbs; i++) { //Declares and initializes arms.
		double length;
		cout << "Length of limb: ";
		cin >> length;
		arm.at(i) = (i == 0) ? Limb(length) : Limb(length, arm.at(i - 1)); 
	}
	// Test Limbs;

	for (int i = 0; i < numLimbs; i++) { //Prints out initial coordinates
		arm.at(i).printCoords();
	}
	cout << "\n\n";

	arm = rotateArm(arm, 1, 120*PI/180); //Rotates the 2nd limb by 120 degrees
	arm = rotateArm(arm, 2, 120*PI/180); //Rotates the 3rd limb by 120 degrees
	//This effectively bends the arm into an equilateral triangle, with the head of the last limb being on the same position as the tail of the first limb.

	for (int i = 0; i < numLimbs; i++) {
		arm.at(i).printCoords(); //Prints Updated Coordinates
	}

	return 0;
}

vector<Limb> rotateArm(vector<Limb> arm, int startJoint, double radians) //This value rotates all limbs starting from startjoin by radian degrees (all limbs attached to start joint are rotated, all previous limbs are not)
{
	Limb joint = arm.at(startJoint); 
	double jointX = joint.getXTail(); //joint x coord
	double jointY = joint.getYTail(); //joint y coord

	//The purpose of this is to make our rotations relative to our joint values, we can then use a rotation matrix to rotate our position vectors

	for (int i = startJoint; i < arm.size(); i++) {
		//Utilizes a rotation matrix in the form x = xosin(theta) - yosin(theta), y = xosin(theta) + yocos(theta)
		Limb limb = arm.at(i);

		double newXHead = (limb.getXHead() - jointX) * cos(radians) - (limb.getYHead() - jointY) * sin(radians) + jointX; //Computes New X Head
		double newYHead = (limb.getXHead() -jointX) * sin(radians) + (limb.getYHead() -jointY) * cos(radians) + jointY; //Computes New Y Head


		double newXTail = (limb.getXTail() - jointX) * cos(radians) - (limb.getYTail() - jointY) * sin(radians) + jointX; //Computes New X Tail
		double newYTail = (limb.getXTail() -jointX) * sin(radians) + (limb.getYTail() -jointY) * cos(radians) + jointY; //Computes New Y Tail


		arm.at(i).updatePosition(newXHead, newYHead, newXTail, newYTail); //Updates Tail Positions
	}
	return arm;
}



