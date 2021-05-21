#include <stdlib.h>
#include <iostream>
#include "CPU.h"
using namespace std;
int main(int argc, char* argv[]) {
	union Register{
		uint16_t reg;
		struct reg8{
			unsigned char lo;
			unsigned char hi;
		} reg8;
	};
	CPU cpu;
	cpu.start(argv[1]);
	system("pause");
}