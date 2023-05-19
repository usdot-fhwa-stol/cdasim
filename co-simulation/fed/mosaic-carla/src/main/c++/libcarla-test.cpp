#include "libcarla-test.hpp"
#include <boost/asio.hpp>
#include "carla/client/Client.h"
#include "carla/Time.h"

int main(int argc, char** argv) {
    std::cout << "Hello world" << std::endl;
    carla::client::Client target_client{"127.0.0.1", 1516, 0};
    target_client.GetWorld().Tick(carla::time_duration::milliseconds(50));
    return 0;
}

