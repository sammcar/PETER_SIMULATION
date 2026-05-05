#include <iostream>
#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <map>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/select.h>

constexpr int PACKET_SIZE = 1400;
constexpr int CAMERA_PORT = 8080;
constexpr int LIDAR_PORT = 8888;

// 📸 Memoria compartida para imágenes
#define SHM_NAME_IMAGE "/shared_image"
#define WIDTH 640
#define HEIGHT 480
#define CHANNELS 3
#define FRAME_SIZE (WIDTH * HEIGHT * CHANNELS)

// 🔺 Memoria compartida para datos del LIDAR
#define SHM_NAME_LIDAR "/shared_lidar"
#define MAX_LIDAR_POINTS 500

// 🔺 Estructura para datos del LIDAR
struct LidarData {
    uint64_t timestamp;
    double angle;
    int distance;
    int intensity;
};

int main() {
    // 🌐 Crear sockets UDP para la cámara y el LiDAR
    int sockfd_camera = socket(AF_INET, SOCK_DGRAM, 0);
    int sockfd_lidar = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_camera < 0 || sockfd_lidar < 0) {
        std::cerr << "❌ Error al crear los sockets UDP." << std::endl;
        return -1;
    }

    // Configurar direcciones de los sockets
    struct sockaddr_in serverAddr_camera{}, serverAddr_lidar{}, clientAddr{};
    socklen_t clientLen = sizeof(clientAddr);

    serverAddr_camera.sin_family = AF_INET;
    serverAddr_camera.sin_addr.s_addr = INADDR_ANY;
    serverAddr_camera.sin_port = htons(CAMERA_PORT);

    serverAddr_lidar.sin_family = AF_INET;
    serverAddr_lidar.sin_addr.s_addr = INADDR_ANY;
    serverAddr_lidar.sin_port = htons(LIDAR_PORT);

    // Enlazar los sockets a los puertos
    if (bind(sockfd_camera, (struct sockaddr*)&serverAddr_camera, sizeof(serverAddr_camera)) < 0 ||
        bind(sockfd_lidar, (struct sockaddr*)&serverAddr_lidar, sizeof(serverAddr_lidar)) < 0) {
        std::cerr << "❌ Error al enlazar los sockets." << std::endl;
        close(sockfd_camera);
        close(sockfd_lidar);
        return -1;
    }

    // 📷 Crear memoria compartida para imágenes
    int shm_fd_img = shm_open(SHM_NAME_IMAGE, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd_img, FRAME_SIZE);
    void* shm_ptr_img = mmap(nullptr, FRAME_SIZE, PROT_WRITE, MAP_SHARED, shm_fd_img, 0);
    close(shm_fd_img);

    // 🔺 Crear memoria compartida para datos del LiDAR
    int shm_fd_lidar = shm_open(SHM_NAME_LIDAR, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd_lidar, sizeof(LidarData) * MAX_LIDAR_POINTS);
    void* shm_ptr_lidar = mmap(nullptr, sizeof(LidarData) * MAX_LIDAR_POINTS, PROT_WRITE, MAP_SHARED, shm_fd_lidar, 0);
    close(shm_fd_lidar);

    if (shm_ptr_img == MAP_FAILED || shm_ptr_lidar == MAP_FAILED) {
        std::cerr << "❌ Error al mapear memoria." << std::endl;
        return -1;
    }

    fd_set readfds;
    int max_sd = std::max(sockfd_camera, sockfd_lidar);
    uchar packet[PACKET_SIZE + 4];

    std::cout << "📡 Publicando imágenes y datos LIDAR en memoria compartida...\n";

    while (true) {
        FD_ZERO(&readfds);
        FD_SET(sockfd_camera, &readfds);
        FD_SET(sockfd_lidar, &readfds);

        struct timeval timeout = {1, 0};
        int activity = select(max_sd + 1, &readfds, nullptr, nullptr, &timeout);
        if (activity < 0) {
            std::cerr << "❌ Error en select()." << std::endl;
            break;
        }

        // 📷 Procesar imagen de la cámara
        if (FD_ISSET(sockfd_camera, &readfds)) {
            ssize_t received_bytes = recvfrom(sockfd_camera, packet, PACKET_SIZE + 4, 0,
                                              (struct sockaddr*)&clientAddr, &clientLen);
            if (received_bytes > 4) {
                cv::Mat img = cv::imdecode(std::vector<uchar>(packet + 4, packet + received_bytes), cv::IMREAD_COLOR);
                if (!img.empty()) {
                    cv::resize(img, img, {WIDTH, HEIGHT});
                    memcpy(shm_ptr_img, img.data, FRAME_SIZE);
                }
            }
        }

        // 🔺 Procesar datos del LIDAR
        if (FD_ISSET(sockfd_lidar, &readfds)) {
            LidarData dataBuffer[MAX_LIDAR_POINTS];
            ssize_t received_bytes = recvfrom(sockfd_lidar, dataBuffer, sizeof(dataBuffer), 0,
                                              (struct sockaddr*)&clientAddr, &clientLen);
            if (received_bytes > 0) {
                int numData = received_bytes / sizeof(LidarData);
                memcpy(shm_ptr_lidar, dataBuffer, numData * sizeof(LidarData));
                std::cout << "📡 Datos LIDAR almacenados: " << numData << " puntos\n";
            }
        }
    }

    // Liberar recursos
    munmap(shm_ptr_img, FRAME_SIZE);
    shm_unlink(SHM_NAME_IMAGE);
    munmap(shm_ptr_lidar, sizeof(LidarData) * MAX_LIDAR_POINTS);
    shm_unlink(SHM_NAME_LIDAR);
    close(sockfd_camera);
    close(sockfd_lidar);

    return 0;
}
