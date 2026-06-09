#include <iostream>
#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <map>
#include <fcntl.h>
#include <sys/mman.h>

// Configuración de memoria compartida
constexpr char SHM_NAME[] = "/dev/shm/shared_image";
constexpr int IMAGE_WIDTH = 640;
constexpr int IMAGE_HEIGHT = 480;
constexpr int IMAGE_CHANNELS = 3;
constexpr size_t FRAME_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS;

// Configuración de puertos
constexpr int PACKET_SIZE = 1400;  // Tamaño de cada paquete UDP
constexpr int CAMERA_PORT = 8080;

// Almacena los fragmentos de imagen recibidos
struct ImageBuffer {
    std::map<uint16_t, std::vector<uchar>> fragments;
    uint16_t total_packets = 0;
    uint16_t received_packets = 0; // Contador de paquetes recibidos
};

int main() {
    std::cout << "\u2705 Iniciando servidor UDP para recibir imagen y escribir en memoria compartida." << std::endl;
    
    // Crear memoria compartida
    int shm_fd = open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) {
        std::cerr << "\u274c Error al abrir la memoria compartida." << std::endl;
        return -1;
    }
    ftruncate(shm_fd, FRAME_SIZE);
    void* shm_ptr = mmap(0, FRAME_SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        std::cerr << "\u274c Error al mapear la memoria compartida." << std::endl;
        return -1;
    }
    std::cout << "\u2705 Memoria compartida configurada correctamente." << std::endl;

    // Crear socket UDP
    int sockfd_camera = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd_camera < 0) {
        std::cerr << "\u274c Error al crear el socket UDP." << std::endl;
        return -1;
    }

    // Configurar dirección del socket
    struct sockaddr_in serverAddr_camera{}, clientAddr{};
    socklen_t clientLen = sizeof(clientAddr);

    serverAddr_camera.sin_family = AF_INET;
    serverAddr_camera.sin_addr.s_addr = INADDR_ANY;
    serverAddr_camera.sin_port = htons(CAMERA_PORT);

    // Enlazar el socket al puerto
    if (bind(sockfd_camera, (struct sockaddr*)&serverAddr_camera, sizeof(serverAddr_camera)) < 0) {
        std::cerr << "\u274c Error al enlazar el socket UDP." << std::endl;
        close(sockfd_camera);
        return -1;
    }
    std::cout << "\u2705 Servidor UDP escuchando en el puerto " << CAMERA_PORT << std::endl;

    // Buffer para recibir datos
    uchar packet[PACKET_SIZE + 4]; // Buffer para recibir datos (4 bytes de encabezado)
    ImageBuffer image_buffer;

    while (true) {
        ssize_t received_bytes = recvfrom(sockfd_camera, packet, PACKET_SIZE + 4, 0,
                                          (struct sockaddr*)&clientAddr, &clientLen);
        if (received_bytes > 4) {
            uint16_t packet_id = packet[0] | (packet[1] << 8);
            uint16_t total_packets = packet[2] | (packet[3] << 8);

            // Inicializar un nuevo buffer si es necesario
            if (image_buffer.total_packets == 0 || image_buffer.total_packets != total_packets) {
                std::cout << "\u267B Nueva imagen detectada. Reiniciando buffer..." << std::endl;
                image_buffer.fragments.clear();
                image_buffer.total_packets = total_packets;
                image_buffer.received_packets = 0;
            }

            // Evitar sobrescribir paquetes duplicados
            if (image_buffer.fragments.find(packet_id) == image_buffer.fragments.end()) {
                image_buffer.fragments[packet_id] = std::vector<uchar>(packet + 4, packet + received_bytes);
                image_buffer.received_packets++;
            }

            // Verificar si se han recibido todos los paquetes
            if (image_buffer.received_packets == total_packets) {
                std::cout << "\u2705 Todos los paquetes recibidos. Ensamblando imagen..." << std::endl;
                
                std::vector<uchar> full_image;
                for (uint16_t i = 0; i < total_packets; i++) {
                    if (image_buffer.fragments.find(i) == image_buffer.fragments.end()) {
                        std::cerr << "\u274c Falta el paquete " << i << ", descartando imagen." << std::endl;
                        image_buffer.fragments.clear();
                        image_buffer.total_packets = 0;
                        image_buffer.received_packets = 0;
                        continue;
                    }
                    full_image.insert(full_image.end(), image_buffer.fragments[i].begin(), image_buffer.fragments[i].end());
                }

                // Intentar decodificar la imagen
                cv::Mat img = cv::imdecode(full_image, cv::IMREAD_COLOR);
                if (!img.empty() && img.total() * img.elemSize() == FRAME_SIZE) {
                    std::cout << "\u2705 Imagen ensamblada correctamente." << std::endl;
                    memcpy(shm_ptr, img.data, FRAME_SIZE);
                    std::cout << "\u2705 Imagen escrita en memoria compartida." << std::endl;
                } else {
                    std::cerr << "\u274c Error al decodificar la imagen." << std::endl;
                }

                // Limpiar buffer después de procesar la imagen
                image_buffer.fragments.clear();
                image_buffer.total_packets = 0;
                image_buffer.received_packets = 0;
            }
        }
    }

    // Cerrar socket y liberar recursos
    close(sockfd_camera);
    munmap(shm_ptr, FRAME_SIZE);
    close(shm_fd);
    std::cout << "\u2705 Recursos liberados correctamente. Saliendo..." << std::endl;
    return 0;
}
