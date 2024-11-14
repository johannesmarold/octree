#ifndef HELPER_H
#define HELPER_H

#include "polyscope/polyscope.h"

#include "polyscope/combining_hash_functions.h"
#include "polyscope/messages.h"

#include "polyscope/file_helpers.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/volume_mesh.h"
#include "polyscope/curve_network.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <random>
#include <string>
#include <utility>
#include <vector>
#include <fstream>
#include <tuple>

#include "args/args.hxx"
#include "portable-file-dialogs.h"

#include <chrono>

using Point = std::array<float, 3>;
using Normal = std::array<float, 3>;

using PointList = std::vector<Point>;

/**
 * struct for collecting data of computation time analysis
 */
struct runtimeAnalytics {
    std::vector<int> numberPoints;
    std::vector<int> numberPointsWithinRadius;
    std::vector<std::chrono::duration<double>> runtimeOctree;
    std::vector<std::chrono::duration<double>> runtimeBruteForce;
    std::vector<float> radius;
};

/**
 * read input file in Off format to a point cloud
 *
 * @param filename name of the OFF file to read from
 * @param points pointer to point Cloud to save the points from the file in
 */
void readOff(std::string const& filename, std::vector<Point>* points);

/**
 * function to write the computation time of the RadiusSearch-method for octree and brute-force
 * over different test Point Clouds
 *
 * @param data collected for runtime analysis
 * @param filename name of the .txt file, where the data is written to
 */
void writeRuntimeAnalyticsToFile(const runtimeAnalytics* data, const std::string& filename);

#endif // HELPER_H
