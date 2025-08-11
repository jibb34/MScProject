#pragma once
#include <algorithm>
#include <string>
#include <utility>

// Returns (line, column) from a byte position in the raw text (1-based).
inline std::pair<size_t, size_t> calc_line_col(const std::string &s,
                                               size_t byte_pos) {
  byte_pos = std::min(byte_pos, s.size());
  size_t line = 1, col = 1;
  for (size_t i = 0; i < byte_pos; ++i) {
    if (s[i] == '\n') {
      ++line;
      col = 1;
    } else {
      ++col;
    }
  }
  return {line, col};
}

// Returns a small window of text around the byte position with a caret.
inline std::string context_snippet(const std::string &s, size_t byte_pos,
                                   size_t window = 60) {
  size_t start = (byte_pos > window ? byte_pos - window : 0);
  size_t end = std::min(s.size(), byte_pos + window);
  std::string snippet = s.substr(start, end - start);

  size_t caret_pos = byte_pos - start;
  std::string caret_line(caret_pos, ' ');
  caret_line.push_back('^');

  return snippet + "\n" + caret_line;
}
