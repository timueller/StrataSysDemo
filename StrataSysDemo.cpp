// StrataSysDemo.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <gtest/gtest.h>

int main(int argc, char** argv)
{
	::testing::InitGoogleTest(&argc, argv);
	int result = RUN_ALL_TESTS();
#ifdef _WIN32
	if (argc < 2 || strcmp(argv[1], "no_pause"))
		system("pause");
#endif
	return result;
}
