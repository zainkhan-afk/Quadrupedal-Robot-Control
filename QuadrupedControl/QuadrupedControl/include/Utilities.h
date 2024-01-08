#ifndef UTILITIES_H
#define UTILITIES_H

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