// SPDX-License-Identifier: Apache-2.0

#include "robot_driver/core/error.hpp"

#include <array>
#include <cstdlib>

namespace
{
struct Translation
{
  ResponseCode code;
  std::string_view zh;
  std::string_view en;
  std::string_view de;
};

constexpr std::array translations{
  Translation{ ResponseCode::OK, "成功", "Success", "Erfolg" },
  Translation{ ResponseCode::INCOMPATIBLE_VERSION, "版本不兼容", "Incompatible version", "Inkompatible Version" },
  Translation{ ResponseCode::CONNECTION_ERROR, "连接错误", "Connection error", "Verbindungsfehler" },
  Translation{ ResponseCode::CONNECTION_TIMEOUT, "连接超时", "Connection timeout",
               "Zeitueberschreitung beim Verbinden" },
  Translation{ ResponseCode::INVALID_IP_ADDRESS, "无效的IP地址", "Invalid IP address", "Ungueltige IP-Adresse" },
  Translation{ ResponseCode::INTERFACE_NOTIMPLEMENTED, "当前控制器该接口未实现", "Interface not implemented",
               "Schnittstelle nicht implementiert" },
  Translation{ ResponseCode::UNSUPPORTED_PARAMETER, "不支持的机器人参数", "Unsupported robot parameter",
               "Nicht unterstuetzter Roboterparameter" },
  Translation{ ResponseCode::INVALID_PARAMETER, "参数有误", "Invalid parameter", "Ungueltiger Parameter" },
  Translation{ ResponseCode::INDEX_OUT_OF_RANGE, "索引下标越界", "Index out of range", "Index ausserhalb des Bereichs" },
  Translation{ ResponseCode::CONTROLLER_ERROR, "控制器错误，详情请联系开发人员", "Controller error",
               "Steuerungsfehler" },
  Translation{ ResponseCode::WAITING_TIMEOUT, "执行等待超时", "Response wait timeout",
               "Zeitueberschreitung beim Warten auf die Antwort" },
  Translation{ ResponseCode::SERVER_ERR, "其他原因", "Other error", "Sonstiger Fehler" },
  Translation{ ResponseCode::CUSTOM_ERROR, "自定义错误", "Custom error", "Benutzerdefinierter Fehler" }
};
}  // namespace

std::string_view ResponseStatus::message(std::string_view language) const
{
  if (language.empty())
  {
    if (const char* configured = std::getenv("ROBOTDRIVERCORE_LANGUAGE"))
      language = configured;
  }
  for (const auto& translation : translations)
  {
    if (translation.code == value)
    {
      if (language == "en")
        return translation.en;
      if (language == "de")
        return translation.de;
      return translation.zh;
    }
  }
  return "Unknown response status";
}
