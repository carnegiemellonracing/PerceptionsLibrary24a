//MLS IMPLEMENTATION


// __global__ void classifyMLSRefined(
//     const Point* points, int* classification, int num_points,
//     float max_slope, float search_radius, float max_distance, float max_height) {

//     int idx = blockIdx.x * blockDim.x + threadIdx.x;
//     if (idx >= num_points) return;

//     Point p = points[idx];

//     // Exclude points far from the car or high above the ground
//     float distance_to_car = sqrtf(p.x * p.x + p.y * p.y);
//     if (distance_to_car > max_distance || p.z > max_height) {
//         classification[idx] = 0; // Not ground
//         return;
//     }

//     // Find neighbors within the search radius
//     float slope_sum = 0;
//     int neighbor_count = 0;

//     for (int i = 0; i < num_points; i++) {
//         if (i == idx) continue;

//         Point neighbor = points[i];

//         float dx = neighbor.x - p.x;
//         float dy = neighbor.y - p.y;
//         float dz = fabsf(neighbor.z - p.z);
//         float distance = sqrtf(dx * dx + dy * dy);

//         if (distance <= search_radius && neighbor.z <= max_height) {
//             float slope = dz / distance;
//             slope_sum += slope;
//             neighbor_count++;
//         }
//     }

//     // Compute the average slope
//     float avg_slope = (neighbor_count > 0) ? slope_sum / neighbor_count : 0;

//     // Classify the point based on the average slope
//     classification[idx] = (avg_slope <= max_slope) ? 1 : 0; // 1: ground, 0: not ground
// }



// thrust::host_vector<Point> GraceAndConrad(
//     const thrust::host_vector<Point>& points,
//     float alpha,
//     int num_bins,
//     float max_slope) {

//     try {
//     auto overall_start = std::chrono::high_resolution_clock::now();

//     int num_points = points.size();

//     // Timer: Copying points to device
//     auto start = std::chrono::high_resolution_clock::now();
//     thrust::device_vector<Point> d_points = points;
//     thrust::device_vector<int> d_classification(num_points, 0);
//     auto end = std::chrono::high_resolution_clock::now();
//     std::cout << "Device memory allocation and copy: "
//               << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";

//     dim3 block(256);
//     dim3 grid((num_points + block.x - 1) / block.x);

//     float search_radius = 0.1f; // Radius for neighbor selection (meters)
//     float max_distance = 60.0f; // Maximum distance from the car (meters)
//     float max_height = 15.0f;  // Maximum allowable height (meters)

//     // Timer: Kernel launch
//     std::cout << "Launching refined MLS kernel...\n";
//     cudaEvent_t kernel_start, kernel_stop;
//     cudaEventCreate(&kernel_start);
//     cudaEventCreate(&kernel_stop);

//     cudaEventRecord(kernel_start);
//     classifyMLSRefined<<<grid, block>>>(
//         thrust::raw_pointer_cast(d_points.data()),
//         thrust::raw_pointer_cast(d_classification.data()),
//         num_points, max_slope, search_radius, max_distance, max_height);
//     cudaEventRecord(kernel_stop);

//     cudaEventSynchronize(kernel_stop);
//     float kernel_time = 0;
//     cudaEventElapsedTime(&kernel_time, kernel_start, kernel_stop);
//     std::cout << "Kernel execution time: " << kernel_time << " ms\n";

//     // Timer: Copying classification results back to host
//     start = std::chrono::high_resolution_clock::now();
//     thrust::host_vector<int> h_classification = d_classification;
//     end = std::chrono::high_resolution_clock::now();
//     std::cout << "Copy results back to host: "
//               << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";

//     // Timer: Filtering ground points
//     start = std::chrono::high_resolution_clock::now();
//     thrust::host_vector<Point> ground_points;
//     for (int i = 0; i < num_points; i++) {
//         if (h_classification[i] == 1) {
//             ground_points.push_back(points[i]);
//         }
//     }
//     end = std::chrono::high_resolution_clock::now();
//     std::cout << "Filter ground points: "
//               << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " ms\n";

//     auto overall_end = std::chrono::high_resolution_clock::now();
//     std::cout << "Overall execution time: "
//               << std::chrono::duration_cast<std::chrono::milliseconds>(overall_end - overall_start).count() << " ms\n";

//     std::cout << "Number of ground points: " << ground_points.size() << "\n";

//     return ground_points;

// } catch (const thrust::system_error& e) {
//         std::cerr << "Thrust system error: " << e.what() << std::endl;
//         return thrust::host_vector<Point>();
//     } catch (const std::exception& e) {
//         std::cerr << "Standard exception: " << e.what() << std::endl;
//         return thrust::host_vector<Point>();
//     } catch (...) {
//         std::cerr << "Unknown error occurred in GraceAndConrad." << std::endl;
//         return thrust::host_vector<Point>();
//     }
// }