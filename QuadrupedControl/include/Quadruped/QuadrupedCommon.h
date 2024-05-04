#ifndef QUADRUPED_COMMON
#define QUADRUPED_COMMON

#ifdef QUADRUPED_LIB
	#define QUADRUPED_API __declspec(dllexport)
	#define QUADRUPED_FUNCTION extern "C" __declspec(dllexport)
#else
	#define QUADRUPED_API __declspec(dllimport)
	#define QUADRUPED_FUNCTION extern "C" __declspec(dllimport)
#endif

#endif //QUADRUPED_COMMON