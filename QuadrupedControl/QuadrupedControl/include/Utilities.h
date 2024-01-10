#ifndef UTILITIES_H
#define UTILITIES_H


#define PI 3.1415926535

int GetLegSign(int leg)
{
	if (leg == 0 || leg == 2)
	{
		return -1;
	}
	else
	{
		return 1;
	}
}

#endif