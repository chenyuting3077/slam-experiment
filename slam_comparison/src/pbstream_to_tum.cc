// Extract trajectory node poses from a cartographer .pbstream file and
// write them out as a TUM-format trajectory file.
#include <cstdio>
#include <fstream>
#include <iostream>

#include "cartographer/common/time.h"
#include "cartographer/io/proto_stream_deserializer.h"
#include "cartographer/mapping/proto/pose_graph.pb.h"

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <in.pbstream> <out_tum.txt>\n";
    return 1;
  }

  const cartographer::mapping::proto::PoseGraph pose_graph =
      cartographer::io::DeserializePoseGraphFromFile(argv[1]);

  std::ofstream out(argv[2]);
  int count = 0;
  for (const auto& trajectory : pose_graph.trajectory()) {
    for (const auto& node : trajectory.node()) {
      const double unix_seconds =
          static_cast<double>(node.timestamp()) / 10000000.0 -
          cartographer::common::kUtsEpochOffsetFromUnixEpochInSeconds;
      const auto& t = node.pose().translation();
      const auto& r = node.pose().rotation();
      char line[256];
      std::snprintf(line, sizeof(line),
                    "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", unix_seconds,
                    t.x(), t.y(), t.z(), r.x(), r.y(), r.z(), r.w());
      out << line;
      ++count;
    }
  }
  std::cerr << "Wrote " << count << " poses to " << argv[2] << "\n";
  return 0;
}
