#include <ros/ros.h>
#include <torch/torch.h>
#include <torch/script.h>
#include <iostream>
#include <memory>
#include <chrono>

int main(int argc, char **argv) {
    ros::init(argc, argv, "sns_vision");
    ros::NodeHandle nh;
    
    int num_warmup = 10;
    int num_samples = 100;

    //Load the network
    std::cout << "Loading Network" << std::endl;
    c10::InferenceMode guard;
    torch::jit::script::Module network;
    try {
    	// Deserialize the ScriptModule from a file using torch::jit::load().
    	network = torch::jit::load("sns_motion_vision.pt");
    }
    catch (const c10::Error& e) {
    	std::cerr << "error loading the model\n";
    	return -1;
    }
    std::cout << "Network loaded successfully" << std::endl;
    
    //Generate a random input
    std::vector<torch::jit::IValue> input;
    input.push_back(torch::rand({24, 64}));
    //std::cout << "Input: " << input << std::endl;
    
    //Execute the network
    //at::Tensor output = network.forward(input).toTensor();
    auto output = network.forward(input);
    //std::cout << "Output: " << output << std::endl;
    
    //Warmup
    std::cout << "Warming up" << std::endl;
    for (int i = 0; i < num_warmup; i++){
    	output = network.forward(input);
    }
    
    //Benchmark
    std::cout << "Benchmarking" << std::endl;
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    for (int i = 0; i < num_samples; i++){
    	auto begin = std::chrono::high_resolution_clock::now();
    	output = network.forward(input).toTensor();
    	auto end = std::chrono::high_resolution_clock::now();
    	std::cout << "Step time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count() << "ms" << std::endl;
    }
    
    
    
    return 0;
}
