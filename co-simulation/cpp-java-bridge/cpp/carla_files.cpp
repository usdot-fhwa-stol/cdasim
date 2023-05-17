#include "carla_files.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }

int dryTest()
{
    try {

        std::string host = "localhost";
        uint16_t port = 2000;
        //std::tie(host, port) = ParseArguments(argc, argv);

        std::mt19937_64 rng((std::random_device())());

        auto client = cc::Client(host, port);
        client.SetTimeout(40s);

        std::cout << "Client API version : " << client.GetClientVersion() << '\n';
        std::cout << "Server API version : " << client.GetServerVersion() << '\n';

        return true;
    } 
    catch (const std::exception &e) 
    {
        std::cout << "\nException: " << e.what() << std::endl;
        return false;
    }
}

int main() {
  std::cout << "\nCalled Main somehow: " << std::endl;
}

