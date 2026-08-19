#include "xr1_moveit_bridge/validator.h"

#include <sstream>
#include <string>

#include <gtest/gtest.h>

TEST(ValidatorContractTest, RejectsMalformedJsonWithoutProcessAbort) {
    std::istringstream input("not-json");
    std::ostringstream output;
    std::ostringstream errors;

    EXPECT_NE(xr1_moveit_bridge::runValidator(input, output, errors), 0);
    EXPECT_TRUE(output.str().empty());
    EXPECT_NE(errors.str().find("invalid_json"), std::string::npos);
}

TEST(ValidatorContractTest, RejectsUnsupportedSchema) {
    std::istringstream input(R"({"schema_version":99})");
    std::ostringstream output;
    std::ostringstream errors;

    EXPECT_NE(xr1_moveit_bridge::runValidator(input, output, errors), 0);
    EXPECT_NE(errors.str().find("unsupported_schema"), std::string::npos);
}
