#include "prototype.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <stdio.h>

int print_hello_world()
{
    printf("Cout: Hello World\n");

    return 1;
}

unsigned int factorial(unsigned int number)
{
    return number <= 1 ? 1 : factorial(number - 1) * number;
}