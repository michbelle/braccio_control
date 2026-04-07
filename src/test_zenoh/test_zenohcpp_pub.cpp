#include "zenoh.hxx"

#include <chrono>
#include <thread>

#include <iostream>

#include <cstring> 

typedef struct position_motor{
    float rot1;
    float arm1_up;
    float arm2_up;
    float arm3_up;
    float rot_grasp;
    float grasp;
} position_motor;


int main(){
    zenoh::Config config = zenoh::Config::create_default();
    // zenoh::Session session = zenoh::Session::open(std::move(config));
    auto session = std::make_unique<zenoh::Session>(zenoh::Session::open(std::move(config)));

    // zenoh::Publisher publisher = session.declare_publisher(zenoh::KeyExpr("braccio_control/positions"));
    auto publisher = std::make_unique<zenoh::Publisher>(session->declare_publisher(zenoh::KeyExpr("braccio_control/positions")));
    // publisher.put(zenoh::ext::serialize(ctrl_position_motor));
    // const std::vector<float> input = {0,1.908,0,2.25,0,5.5};
    position_motor input = {0,1.908,0,2.25,0,5.5};

    // std::vector<uint8_t> buffer(sizeof(input));
    while(true){
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        auto serializer = zenoh::ext::Serializer();
        serializer.serialize(input.rot1);
        serializer.serialize(input.arm1_up);
        serializer.serialize(input.arm2_up);
        serializer.serialize(input.arm3_up);
        serializer.serialize(input.rot_grasp);
        serializer.serialize(input.grasp);
        zenoh::Bytes bytes = std::move(serializer).finish();
        publisher->put(std::move(bytes));
    }
}
