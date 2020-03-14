
#ifndef UNITY_API
#define UNITY_API

#ifdef WIN64
#define UNITY_API __declspec(dllimport) __stdcall
#else
#define UNITY_API
#endif

#endif