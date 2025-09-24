#include <iostream>
#include <opencv2/opencv.hpp>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <map>
#include <sys/select.h>

// Configuración de puertos
constexpr int PACKET_SIZE = 1400;  // Tamaño de cada paquete UDP
constexpr int CAMERA_PORT = 8080;
constexpr size_t MAX_BUFFER_SIZE = 400000;  // Evita desbordes en el buffer de imagen

// Almacena los fragmentos de imagen recibidos
struct ImageBuffer {
    std::vector<uchar> data;
    uint16_t total_packets = 0;
    std::map<uint16_t, std::vector<uchar>> fragments;
};

int main() {
    // Crear socket UDP para la Cámara
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

    // Configuración de `select()` para manejar el socket
    fd_set readfds;
    uchar packet[PACKET_SIZE + 4]; // Buffer para recibir datos (4 bytes de encabezado)

    // Buffer para reconstrucción de imagen
    ImageBuffer image_buffer;

    while (true) {
        FD_ZERO(&readfds);
        FD_SET(sockfd_camera, &readfds);

        struct timeval timeout = {1, 0}; // Evita bloqueos en `select()`
        int activity = select(sockfd_camera + 1, &readfds, nullptr, nullptr, &timeout);
        if (activity < 0) {
            std::cerr << "\u274c Error en select()." << std::endl;
            break;
        }

        // \ud83d\udcf7 Recibir datos de la cámara
        if (FD_ISSET(sockfd_camera, &readfds)) {
            ssize_t received_bytes = recvfrom(sockfd_camera, packet, PACKET_SIZE + 4, 0,
                                              (struct sockaddr*)&clientAddr, &clientLen);
            if (received_bytes > 4) {
                uint16_t packet_id = packet[0] | (packet[1] << 8);
                uint16_t total_packets = packet[2] | (packet[3] << 8);

                if (image_buffer.total_packets == 0 || image_buffer.total_packets != total_packets) {
                    // Nueva imagen detectada, limpiar buffer
                    image_buffer.fragments.clear();
                    image_buffer.total_packets = total_packets;
                }

                // Guardar fragmento en el buffer
                std::vector<uchar> fragment(packet + 4, packet + received_bytes);
                image_buffer.fragments[packet_id] = fragment;

                // Si hemos recibido todos los paquetes, ensamblar la imagen
                if (image_buffer.fragments.size() == total_packets) {
                    std::vector<uchar> full_image;
                    for (uint16_t i = 0; i < total_packets; i++) {
                        full_image.insert(full_image.end(), image_buffer.fragments[i].begin(), image_buffer.fragments[i].end());
                    }

                    // Intentar decodificar la imagen
                    cv::Mat img = cv::imdecode(full_image, cv::IMREAD_COLOR);
                    if (!img.empty()) {
                        cv::imshow("Imagen recibida", img);
                    } else {
                        std::cerr << "\u274c Error al decodificar la imagen." << std::endl;
                    }

                    // Limpiar buffer después de mostrar la imagen
                    image_buffer.fragments.clear();
                    image_buffer.total_packets = 0;
                }
            }
        }

        if (cv::waitKey(1) == 'q') break;
    }

    // Cerrar socket y liberar recursos
    close(sockfd_camera);
    cv::destroyAllWindows();
    return 0;
}