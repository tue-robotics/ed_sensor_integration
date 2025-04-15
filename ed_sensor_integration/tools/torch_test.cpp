#include <torch/torch.h>
#include <iostream>

int main() {
    torch::Tensor tensor = torch::eye(3).cuda();
    std::cout << tensor << std::endl;
}
