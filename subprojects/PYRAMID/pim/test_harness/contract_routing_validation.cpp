#include <pcl/pcl_executor.h>
#include <pcl/pcl_transport_routing.h>

#include <cstdio>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: %s <routing-manifest>\n", argv[0]);
        return 2;
    }

    pcl_executor_t* executor = pcl_executor_create();
    if (!executor) {
        std::fprintf(stderr, "failed to create executor\n");
        return 2;
    }

    pcl_transport_routing_t* routing = nullptr;
    char diag[512] = "";
    const pcl_status_t rc =
        pcl_transport_routing_load(executor, argv[1], &routing, diag, sizeof(diag));
    if (rc != PCL_OK) {
        std::fprintf(stderr, "routing validation failed rc=%d diag=%s\n",
                     static_cast<int>(rc), diag);
        pcl_executor_destroy(executor);
        return 1;
    }

    const size_t transports = pcl_transport_routing_transport_count(routing);
    std::printf("contract_routing_validation=pass transports=%zu\n", transports);
    pcl_transport_routing_destroy(routing);
    pcl_executor_destroy(executor);
    return transports == 2U ? 0 : 1;
}
