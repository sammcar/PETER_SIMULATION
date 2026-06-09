#include <opencv2/opencv.hpp>
#include <iostream>
#include <arpa/inet.h>
#include <unistd.h>
#include <vector>
#include <cstring>
#include <csignal>
#include <sys/mman.h>
#include <fcntl.h>

constexpr int SERVER_PORT = 8080;
constexpr size_t CHUNK_SIZE = 4096; // Tamaño del fragmento
constexpr int BACKLOG = 1; // Número de conexiones en espera
constexpr int IMAGE_WIDTH = 640;
constexpr int IMAGE_HEIGHT = 480;
constexpr int IMAGE_CHANNELS = 3;
constexpr size_t FRAME_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS;
constexpr char SHM_NAME[] = "/dev/shm/shared_image";

int sockfd = -1, clientfd = -1;
int shm_fd = -1;
void* shm_ptr = nullptr;

// Manejador de señal para cerrar los sockets y liberar recursos
void signal_handler(int) {
    if (clientfd >= 0) {
        close(clientfd);
        std::cout << "\n🔴 Cliente desconectado." << std::endl;
    }
    if (sockfd >= 0) {
        close(sockfd);
        std::cout << "🔴 Servidor cerrado correctamente." << std::endl;
    }
    if (shm_fd >= 0) {
        munmap(shm_ptr, FRAME_SIZE);
        close(shm_fd);
        std::cout << "🗑️ Memoria compartida liberada." << std::endl;
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

// Configurar memoria compartida
bool setup_shared_memory() {
    shm_fd = open(SHM_NAME, O_RDWR | O_CREAT, 0666);
    if (shm_fd < 0) {
        perror("❌ Error al abrir memoria compartida");
        return false;
    }

    if (ftruncate(shm_fd, FRAME_SIZE) < 0) {
        perror("❌ Error al truncar memoria compartida");
        return false;
    }

    shm_ptr = mmap(nullptr, FRAME_SIZE, PROT_WRITE, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) {
        perror("❌ Error al mapear memoria compartida");
        return false;
    }

    std::cout << "✅ Memoria compartida configurada en " << SHM_NAME << std::endl;
    return true;
}

// Recibir una imagen del cliente y escribirla en la memoria compartida
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

    // Asegurar que la imagen tenga el tamaño esperado
    cv::resize(img, img, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT));

    // Escribir la imagen en la memoria compartida
    std::memcpy(shm_ptr, img.data, FRAME_SIZE);

    std::cout << "💾 Imagen almacenada en memoria compartida." << std::endl;
    return true;
}

int main() {
    signal(SIGINT, signal_handler);

    if (!setup_shared_memory()) return -1;

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
