#include <iostream>
#include <stdio.h> //print f
#include <math.h> //trig functions
#include <vector> //vector 
#include <iomanip> //set precision for double input

#define PI 3.14159265

using namespace std;

class Limb
{
	public:
		Limb();
		Limb(double);
		Limb(double, Limb);
		double getXHead();
		double getYHead();
		double getYTail();
		double getXTail();
		void updatePosition(double, double, double, double);
		void printCoords();
	private:
		double xHead;
		double yHead;
		double xTail;
		double yTail;
		double rotation;
		double length;

};
Limb::Limb()
{
}

Limb::Limb(double length)
{
	yTail = 0; 
	xTail = 0;
	xHead = length; 
	yHead = 0;
	rotation = 0;
	this->length = length;
}

Limb::Limb(double length, Limb limb)
{
	yTail = limb.getYHead();
	xTail = limb.getXHead();
	yHead = yTail;
	xHead = xTail + length;
	rotation = 0;
	this->length = length;
}

double Limb::getXHead()
{
	return xHead;
}

double Limb::getYHead()
{
	return yHead;
}

double Limb::getXTail()
{
	return xTail;
}

double Limb::getYTail()
{
	return yTail;
}

void Limb::updatePosition(double xHead, double yHead, double xTail, double yTail)
{
	this->xHead = xHead;
	this->yHead = yHead;
	this->xTail = xTail;
	this->yTail = yTail;
}

void Limb::printCoords()
{
	printf("Tail:(%.2f,%.2f)\tHead:(%.2f,%.2f)\tVector:(%.2f,%.2f)\n",xTail,yTail,xHead,yHead,(xHead-xTail),(yHead-yTail));
}

vector<Limb> rotateArm(vector<Limb>, int, double);


int main()
{
	int numLimbs;

	cout << "How many limbs do you have: ";
	cin >> numLimbs;
	vector<Limb> arm(numLimbs);
	for (int i = 0; i < numLimbs; i++) {
		double length;
		cout << "Length of limb: ";
		cin >> length;
		arm.at(i) = (i == 0) ? Limb(length) : Limb(length, arm.at(i - 1));
	}
	// Test Limbs;

	for (int i = 0; i < numLimbs; i++) {
		arm.at(i).printCoords();
	}
	cout << "\n\n";
	arm = rotateArm(arm, 1, 120*PI/180);
	arm = rotateArm(arm, 2, 120*PI/180);
	for (int i = 0; i < numLimbs; i++) {
		arm.at(i).printCoords();
	}

	return 0;
}

vector<Limb> rotateArm(vector<Limb> arm, int startJoint, double radians)
{
	Limb joint = arm.at(startJoint);
	double jointX = joint.getXTail();
	double jointY = joint.getYTail();

	for (int i = startJoint; i < arm.size(); i++) {
		Limb limb = arm.at(i);

		double newXTail = limb.getXTail();
		double newYTail = limb.getYTail();

		double newXHead = (limb.getXHead() - jointX) * cos(radians) - (limb.getYHead() - jointY) * sin(radians) + jointX;
		double newYHead = (limb.getXHead() -jointX) * sin(radians) + (limb.getYHead() -jointY) * cos(radians) + jointY;

		if (i != startJoint) {

			newXTail = (limb.getXTail() - jointX) * cos(radians) - (limb.getYTail() - jointY) * sin(radians) + jointX;
			newYTail = (limb.getXTail() -jointX) * sin(radians) + (limb.getYTail() -jointY) * cos(radians) + jointY;

		}

		arm.at(i).updatePosition(newXHead, newYHead, newXTail, newYTail);
	}
	return arm;
}



