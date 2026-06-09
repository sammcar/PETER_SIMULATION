#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/select.h>

constexpr int LIDAR_PORT = 8888;

// 🔹 Memoria compartida para datos del LIDAR
#define SHM_NAME_LIDAR "/shared_lidar"
#define MAX_LIDAR_POINTS 500

// 🔹 Estructura para datos del LIDAR
struct LidarData {
    uint64_t timestamp;
    double angle;
    int distance;
    int intensity;
};

int main() {
    // 🌐 Crear socket UDP para el LiDAR
    int sockfd_lidar = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_lidar < 0) {
        std::cerr << "❌ Error al crear el socket UDP del LiDAR." << std::endl;
        return -1;
    }

    // Configurar dirección del socket
    struct sockaddr_in serverAddr_lidar{}, clientAddr{};
    socklen_t clientLen = sizeof(clientAddr);

    serverAddr_lidar.sin_family = AF_INET;
    serverAddr_lidar.sin_addr.s_addr = INADDR_ANY;
    serverAddr_lidar.sin_port = htons(LIDAR_PORT);

    // Enlazar el socket al puerto
    if (bind(sockfd_lidar, (struct sockaddr*)&serverAddr_lidar, sizeof(serverAddr_lidar)) < 0) {
        std::cerr << "❌ Error al enlazar el socket del LiDAR." << std::endl;
        close(sockfd_lidar);
        return -1;
    }

    // 🔹 Crear memoria compartida para datos del LiDAR
    int shm_fd_lidar = shm_open(SHM_NAME_LIDAR, O_CREAT | O_RDWR, 0666);
    ftruncate(shm_fd_lidar, sizeof(LidarData) * MAX_LIDAR_POINTS);
    void* shm_ptr_lidar = mmap(nullptr, sizeof(LidarData) * MAX_LIDAR_POINTS, PROT_WRITE, MAP_SHARED, shm_fd_lidar, 0);
    close(shm_fd_lidar);

    if (shm_ptr_lidar == MAP_FAILED) {
        std::cerr << "❌ Error al mapear memoria compartida del LiDAR." << std::endl;
        return -1;
    }

    std::cout << "📡 Publicando datos LIDAR en memoria compartida..." << std::endl;

    while (true) {
        LidarData dataBuffer[MAX_LIDAR_POINTS];
        ssize_t received_bytes = recvfrom(sockfd_lidar, dataBuffer, sizeof(dataBuffer), 0,
                                          (struct sockaddr*)&clientAddr, &clientLen);
        if (received_bytes > 0) {
            int numData = received_bytes / sizeof(LidarData);
            memcpy(shm_ptr_lidar, dataBuffer, numData * sizeof(LidarData));
            std::cout << "📡 Datos LIDAR almacenados: " << numData << " puntos" << std::endl;
        }
    }

    // Liberar recursos
    munmap(shm_ptr_lidar, sizeof(LidarData) * MAX_LIDAR_POINTS);
    shm_unlink(SHM_NAME_LIDAR);
    close(sockfd_lidar);

    return 0;
}
