// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>
#include <string_view>
#include <vector>

namespace rmi::detail
{

class LineBuffer
{
public:
  std::vector<std::string> append(const std::string_view bytes)
  {
    buffer_.append(bytes);
    std::vector<std::string> lines;

    std::size_t pos = 0;
    for (std::size_t end_pos = buffer_.find("\r\n", pos); end_pos != std::string::npos;
         end_pos = buffer_.find("\r\n", pos))
    {
      lines.emplace_back(buffer_.substr(pos, end_pos - pos));
      pos = end_pos + 2;
    }

    buffer_.erase(0, pos);
    return lines;
  }

private:
  std::string buffer_;
};

}  // namespace rmi::detail
