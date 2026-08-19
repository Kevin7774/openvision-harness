#include "xr1_moveit_bridge/validator.h"

#include <iostream>

int main() {
    return xr1_moveit_bridge::runValidator(std::cin, std::cout, std::cerr);
}
