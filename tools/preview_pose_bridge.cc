/// Move the preview camera without paying gz's CLI cost.
///
/// 'gz service' costs ~400 ms per call on this box (process start + transport
/// discovery), which turned every drag step into a 0.4 s jump.  This helper
/// keeps a single transport node alive and reads poses from stdin, so after the
/// first request a pose costs about 1 ms.
///
/// Protocol: one pose per line,
///   x y z qx qy qz qw
/// EOF on stdin exits the process.

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/transport/Node.hh>

#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc < 2) {
    std::cerr << "usage: preview_pose_bridge <world> [model]" << std::endl;
    return 2;
  }
  const std::string world = argv[1];
  const std::string model = argc > 2 ? argv[2] : "preview_cam";
  const std::string service = "/world/" + world + "/set_pose";

  gz::transport::Node node;
  gz::msgs::Boolean reply;
  bool result = false;

  double x, y, z, qx, qy, qz, qw;
  while (std::cin >> x >> y >> z >> qx >> qy >> qz >> qw) {
    gz::msgs::Pose req;
    req.set_name(model);
    req.mutable_position()->set_x(x);
    req.mutable_position()->set_y(y);
    req.mutable_position()->set_z(z);
    req.mutable_orientation()->set_x(qx);
    req.mutable_orientation()->set_y(qy);
    req.mutable_orientation()->set_z(qz);
    req.mutable_orientation()->set_w(qw);
    if (!node.Request(service, req, 500, reply, result) || !result) {
      std::cerr << "set_pose failed" << std::endl;
    }
  }
  return 0;
}
