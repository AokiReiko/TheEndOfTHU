
// Collect data from Unreal Engine by program

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <cmath>

int MAX_FLOAT = Utils::max<float>();

/*

虚幻引擎3支持的最大的世界尺寸是512k x 512k (524288 x 524288) 个虚幻单位。
如果按照默认的引擎单位1UU=2cm(10km x 10km)计算，这大约近似等于10平方千米。
游戏性区域的实际尺寸通常远比这个尺寸小，一般基于地形的室外地图的尺寸少于1平方千米。
注意，那些放置在最大世界尺寸外面的几何体，比如天空球体，仍然是进行渲染的，但是“世界”几何体必须在这个总的可用区域之内。
*/
class Camera {
private:
    string path;

public:
    Camera(std::string path_) {
        path = path_;
    }
    ~Camera() {}
    bool getImage(int index) {

        /* 
        camera_name: 0"-right, "1"-left 
        */
        vector<ImageRequest> request = { 
            //right png format
            ImageRequest("0", ImageType::Scene),
            ImageRequest("0", ImageType::Segmentation),
            //left png format
            ImageRequest("1", ImageType::Scene),
            ImageRequest("1", ImageType::Segmentation),
        };

        const vector<ImageResponse>& response = client.simGetImages(request);
        std::cout << "# of images received: " << response.size() << std::endl;

        if (response.size() != request.size()) return false;

        if (path != "") {
            std::string file_path = FileSystem::combine(path, std::to_string(index));
            std::ofstream file(file_path + "_right.png", std::ios::binary);
            file.write(reinterpret_cast<const char*>(response.at(0).image_data_uint8.data()), image_info.image_data_uint8.size());
            file.close();

            std::ofstream file(file_path + "_right_m.png", std::ios::binary);
            file.write(reinterpret_cast<const char*>(response.at(1).image_data_uint8.data()), image_info.image_data_uint8.size());
            file.close();

            std::ofstream file(file_path + "_left.png", std::ios::binary);
            file.write(reinterpret_cast<const char*>(response.at(2).image_data_uint8.data()), image_info.image_data_uint8.size());
            file.close();

            std::ofstream file(file_path + "_left_m.png", std::ios::binary);
            file.write(reinterpret_cast<const char*>(response.at(3).image_data_uint8.data()), image_info.image_data_uint8.size());
            file.close();
        } 
        return true;
    }
};



int main() 
{
    using namespace msr::airlib;

    msr::airlib::MultirotorRpcLibClient client;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    int height = 20;
    int start_x = 0, start_y = 0;
    int end_x = 0, end_y = 0;
    int dataset_size = 0;
    std::string path;


    
    try {
        client.confirmConnection();

        std::cout << "Arm and control the drone." << std::endl;
        client.enableApiControl(true);
        client.armDisarm(true);


        std::cout << "height: " << std::endl; std::cin >> height;
        std::cout << "start point (2D/m): " << std::endl; std::cin >> start_x >> start_y;
        std::cout << "end point (2D/m): " << std::endl; std::cin >> end_x >> end_y;
        std::cout << "dataset size: " << std::endl; std::cin >> dataset_size;
        std::cout << "Enter path with ending separator to save images: " << std::endl; std::getline(std::cin, path);

        Camera camera(path);

        if (start_x == end_x || start_y == end_y) return 0;

        if (start_x > end_x) {
            int temp = end_x;
            end_x = start_x;
            start_x = temp;
        }

        if (start_y > end_y) {
            int temp = end_y;
            end_y = start_y;
            start_y = temp;
        }

        if (dataset_size <= 0) dataset_size = 100;


        /* 需要换算成m */
      
        std::cout << "Press Enter to takeoff" << std::endl; std::cin.get();
        float takeoffTimeout = 5; 
        client.takeoffAsync(takeoffTimeout)->waitOnLastTask();

        // switch to explicit hover mode so that this is the fall back when 
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hoverAsync()->waitOnLastTask();

        std::cout << "Press Enter to fly in a 10m box pattern at 3 m/s velocity" << std::endl; std::cin.get();
        // moveByVelocityZ is an offboard operation, so we need to set offboard mode.
        client.enableApiControl(true); 
        auto position = client.getMultirotorState().getPosition();
        float z = position.z(); // current position (NED coordinate system).  

        float speed = 3.0f;
        float stride = (end_y - start_y) * (end_x - start_x) / dataset_size; 
        stride = std::sqrt(stride);

        float duration = stride / speed;

        // Go to start point
        client.moveToPositionAsync(start_x, start_y, height, speed, MAX_FLOAT, driveTrain, yaw_mode);
        
        DrivetrainType driveTrain = DrivetrainType::ForwardOnly;
        YawMode yaw_mode(true, 0);

        float next_x = start_x, next_y = start_y;
        int index = 0;
        float delta_y = stride;
        while (next_x < end_x) {
            while (next_y < end_y) {
                camera.getImage(index++);
                next_y += delta_y;
                moveToPositionAsync(next_x, next_y, height, speed, MAX_FLOAT, driveTrain, yaw_mode);
            }
            next_y -= delta_y;
            delta_y = -delta_y;
            next_x += stride;
            moveToPositionAsync(next_x, next_y, height, speed, MAX_FLOAT, driveTrain, yaw_mode);
        }
       
        client.landAsync()->waitOnLastTask();
        client.armDisarm(false);

    }
    catch (rpc::rpc_error&  e) {
        std::string msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl << msg << std::endl;
    }

    return 0;
}