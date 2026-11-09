// SPDX-License-Identifier: Apache-2.0

#include <gtest/gtest.h>

#include <array>
#include <cstdlib>
#include <optional>
#include <string>

#include "robot_driver/core/error.hpp"

TEST(ResponseStatus, CodesAndTranslations)
{
  const ResponseStatus success;
  EXPECT_TRUE(success.ok());
  EXPECT_EQ(success.code(), 0);
  EXPECT_EQ(success.message("zh"), "成功");
  EXPECT_EQ(success.message("en"), "Success");
  EXPECT_EQ(success.message("de"), "Erfolg");
  EXPECT_EQ(success.message("unsupported"), success.message("zh"));
  const ResponseStatus error{ ResponseCode::CONTROLLER_ERROR, "FANUC detail", 2556940 };
  EXPECT_FALSE(error.ok());
  EXPECT_EQ(error.code(), 3000);
  EXPECT_EQ(error.message("en"), "Controller error");
  EXPECT_EQ(error.controller_error, 2556940);
  EXPECT_EQ(error.detail, "FANUC detail");

  constexpr std::array codes{ ResponseCode::OK,
                              ResponseCode::INCOMPATIBLE_VERSION,
                              ResponseCode::CONNECTION_ERROR,
                              ResponseCode::CONNECTION_TIMEOUT,
                              ResponseCode::INVALID_IP_ADDRESS,
                              ResponseCode::INTERFACE_NOTIMPLEMENTED,
                              ResponseCode::UNSUPPORTED_PARAMETER,
                              ResponseCode::INVALID_PARAMETER,
                              ResponseCode::INDEX_OUT_OF_RANGE,
                              ResponseCode::CONTROLLER_ERROR,
                              ResponseCode::WAITING_TIMEOUT,
                              ResponseCode::SERVER_ERR };
  for (auto code : codes)
    for (auto language : { "zh", "en", "de" })
      EXPECT_NE(ResponseStatus({ code, {}, std::nullopt }).message(language), "Unknown response status");
}

TEST(ResponseStatus, EnvironmentLanguageIsReadOnEachCall)
{
  struct RestoreLanguage
  {
    std::optional<std::string> original;
    RestoreLanguage()
    {
      if (const char* value = std::getenv("ROBOTDRIVERCORE_LANGUAGE"))
        original = value;
    }
    ~RestoreLanguage()
    {
      if (original)
        setenv("ROBOTDRIVERCORE_LANGUAGE", original->c_str(), 1);
      else
        unsetenv("ROBOTDRIVERCORE_LANGUAGE");
    }
  } restore;

  const ResponseStatus success;
  ASSERT_EQ(unsetenv("ROBOTDRIVERCORE_LANGUAGE"), 0);
  EXPECT_EQ(success.message(), success.message("zh"));
  for (auto language : { "en", "de", "zh", "unsupported" })
  {
    ASSERT_EQ(setenv("ROBOTDRIVERCORE_LANGUAGE", language, 1), 0);
    EXPECT_EQ(success.message(), success.message(language));
  }
}

TEST(ResponseStatus, FalseAndZeroRemainSuccessfulData)
{
  const Response<bool> boolean{ {}, false };
  ASSERT_TRUE(boolean.ok());
  EXPECT_FALSE(*boolean.data);
  const Response<int> number{ {}, 0 };
  ASSERT_TRUE(number.ok());
  EXPECT_EQ(*number.data, 0);
}
