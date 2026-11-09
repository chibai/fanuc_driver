// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <stdexcept>

#include "drivers/fanuc/detail/response.hpp"

TEST(ResponseStatus, OnlyOperationalErrorsAreConverted)
{
  const auto invalid = rmi::detail::captureResponse<int>([]() -> int { throw std::invalid_argument("invalid input"); });
  EXPECT_EQ(invalid.status.code(), 1160);
  EXPECT_EQ(invalid.status.detail, "invalid input");
  EXPECT_FALSE(invalid.data);
  EXPECT_THROW(rmi::detail::captureResponse<int>([]() -> int { throw std::logic_error("programming error"); }),
               std::logic_error);
}

TEST(ResponseStatus, ExpectedPreservesValuesAndTypedErrors)
{
  const auto boolean = rmi::detail::captureExpected<bool>([] { return false; });
  ASSERT_TRUE(boolean.has_value());
  EXPECT_FALSE(*boolean);
  const auto number = rmi::detail::captureExpected<int>([] { return 0; });
  ASSERT_TRUE(number.has_value());
  EXPECT_EQ(*number, 0);
  const auto error = rmi::detail::captureExpected<int>(
      []() -> int { throw rmi::detail::ResponseError({ ResponseCode::CONTROLLER_ERROR, "controller failure", 42 }); });
  ASSERT_FALSE(error.has_value());
  EXPECT_EQ(error.error().code(), 3000);
  EXPECT_EQ(error.error().controller_error, 42);
  EXPECT_EQ(error.error().detail, "controller failure");
  const auto invalid = rmi::detail::captureExpected<int>([]() -> int { throw std::invalid_argument("invalid input"); });
  ASSERT_FALSE(invalid.has_value());
  EXPECT_EQ(invalid.error().value, ResponseCode::INVALID_PARAMETER);
  EXPECT_THROW(rmi::detail::captureExpected<int>([]() -> int { throw std::logic_error("programming error"); }),
               std::logic_error);
}
