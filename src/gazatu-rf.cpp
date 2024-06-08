#include "gazatu-rf.hpp"

namespace tser {
void operator<<(const rf::string& string, tser::BinaryArchive& bin) {
  std::string buf;
  buf.resize(256);
  auto buf_len = unishox2_compress_simple(string.data(), string.size(), buf.data());
  buf.resize(buf_len);

  bin.save(buf);
}

void operator>>(rf::string& string, tser::BinaryArchive& bin) {
  std::string buf;
  bin.load(buf);

  string.resize(256);
  auto buf_len = unishox2_decompress_simple(buf.data(), buf.size(), string.data());
  string.resize(buf_len);
}
} // namespace tser
