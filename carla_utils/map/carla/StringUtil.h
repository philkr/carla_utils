// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <algorithm>
#include <regex>

namespace carla {

  class StringUtil {
  public:

    static const char *ToConstCharPtr(const char *str) {
      return str;
    }

    template <typename StringT>
    static const char *ToConstCharPtr(const StringT &str) {
      return str.c_str();
    }

    template <typename Range1T, typename Range2T>
    static bool StartsWith(const Range1T &input, const Range2T &test) {
      size_t n = std::end(test)-std::begin(test);
      if (std::end(input)-std::begin(input) < n)
        return false;
      return std::equal(std::begin(input), std::begin(input)+n, std::begin(test), std::end(test));
    }

    template <typename Range1T, typename Range2T>
    static bool EndsWith(const Range1T &input, const Range2T &test) {
      size_t n = std::end(test)-std::begin(test);
      if (std::end(input)-std::begin(input) < n)
        return false;
      return std::equal(std::end(input)-n, std::end(input), std::begin(test), std::end(test)+n);
    }

    template <typename WritableRangeT>
    static void ToLower(WritableRangeT &str) {
      std::transform(std::begin(str), std::end(str), std::begin(str), [](int c) {return std::tolower(c);});
    }

    template <typename SequenceT>
    static auto ToLowerCopy(SequenceT str) {
      ToLower(str);
      return str;
    }

    template <typename WritableRangeT>
    static void ToUpper(WritableRangeT &str) {
      std::transform(std::begin(str), std::end(str), std::begin(str), [](int c) {return std::toupper(c);});
    }

    template <typename SequenceT>
    static auto ToUpperCopy(SequenceT str) {
      ToUpper(str);
      return str;
    }

    template <typename WritableRangeT>
    static void lTrim(WritableRangeT &s) {
        s.erase(std::begin(s), std::find_if(std::begin(s), std::end(s), [](int c) {return !std::isspace(c);}));
    }

    template <typename WritableRangeT>
    static void rTrim(WritableRangeT &s) {
        s.erase(std::find_if(std::rbegin(s), std::rend(s), [](int c) {return !std::isspace(c);}).base(), std::end(s));
    }

    template <typename WritableRangeT>
    static void Trim(WritableRangeT &str) {
      rTrim(str);
      lTrim(str);
    }

    template <typename SequenceT>
    static auto TrimCopy(SequenceT str) {
      Trim(str);
      return str;
    }

    template<typename Container, typename Range1T, typename Range2T>
    static void Split(Container &destination, const Range1T &str, const Range2T &separators) {
      std::regex re(std::string("[")+separators+std::string("]+"));
      std::sregex_token_iterator first{str.begin(), str.end(), re, -1}, last;
      destination = Container(first, last);
    }

    /// Match @a str with the Unix shell-style @a wildcard_pattern.
    static bool Match(const char *str, const char *wildcard_pattern);

    /// Match @a str with the Unix shell-style @a wildcard_pattern.
    template <typename String1T, typename String2T>
    static bool Match(const String1T &str, const String2T &wildcard_pattern) {
      return Match(ToConstCharPtr(str), ToConstCharPtr(wildcard_pattern));
    }
  };

} // namespace carla
