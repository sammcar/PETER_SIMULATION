#include <opencv2/opencv.hpp>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <csignal>

constexpr int SERVER_PORT = 8080;
constexpr size_t CHUNK_SIZE = 4096; // Tamaño del fragmento
constexpr int BACKLOG = 1; // Número de conexiones en espera

int sockfd = -1, clientfd = -1;

// Manejador de señal para cerrar los sockets al salir
void signal_handler(int) {
    if (clientfd >= 0) {
        close(clientfd);
        std::cout << "\n🔴 Cliente desconectado." << std::endl;
    }
    if (sockfd >= 0) {
        close(sockfd);
        std::cout << "🔴 Servidor cerrado correctamente." << std::endl;
    }
    exit(0);
}

// Configurar el servidor TCP
int setup_tcp_server(int port) {
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        perror("❌ Error al crear el socket TCP");
        return -1;
    }

    struct sockaddr_in serverAddr{};
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(port);
    serverAddr.sin_addr.s_addr = INADDR_ANY;

    // Permitir reutilizar el puerto
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    if (bind(sockfd, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("❌ Error al enlazar el socket");
        close(sockfd);
        return -1;
    }

    if (listen(sockfd, BACKLOG) < 0) {
        perror("❌ Error en listen()");
        close(sockfd);
        return -1;
    }

    std::cout << "✅ Servidor escuchando en el puerto " << SERVER_PORT << std::endl;
    return sockfd;
}

// Recibir una imagen del cliente
bool receive_frame(int clientfd) {
    uint32_t img_size_network;
    if (recv(clientfd, &img_size_network, sizeof(img_size_network), 0) <= 0) {
        std::cerr << "❌ Error al recibir el tamaño de la imagen." << std::endl;
        return false;
    }

    uint32_t img_size = ntohl(img_size_network);
    std::cout << "📥 Recibiendo imagen de " << img_size << " bytes." << std::endl;

    std::vector<uchar> buffer(img_size);
    size_t bytes_received = 0;

    while (bytes_received < img_size) {
        size_t chunk_size = std::min(CHUNK_SIZE, img_size - bytes_received);
        ssize_t received = recv(clientfd, buffer.data() + bytes_received, chunk_size, 0);

        if (received <= 0) {
            std::cerr << "❌ Error al recibir un fragmento de la imagen." << std::endl;
            return false;
        }
        bytes_received += received;
    }

    // Decodificar la imagen recibida
    cv::Mat img = cv::imdecode(buffer, cv::IMREAD_COLOR);
    if (img.empty()) {
        std::cerr << "❌ Error al decodificar la imagen." << std::endl;
        return false;
    }

    cv::imshow("📸 Imagen Recibida", img);
    cv::waitKey(1);
    return true;
}

int main() {
    signal(SIGINT, signal_handler);

    sockfd = setup_tcp_server(SERVER_PORT);
    if (sockfd < 0) return -1;

    while (true) {
        struct sockaddr_in clientAddr{};
        socklen_t clientLen = sizeof(clientAddr);
        clientfd = accept(sockfd, (struct sockaddr*)&clientAddr, &clientLen);
        if (clientfd < 0) {
            perror("❌ Error al aceptar la conexión");
            continue;
        }

        std::cout << "✅ Cliente conectado." << std::endl;

        while (receive_frame(clientfd));

        std::cout << "🔴 Cliente desconectado." << std::endl;
        close(clientfd);
        clientfd = -1;
    }

    signal_handler(0);
    return 0;
}
