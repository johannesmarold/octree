#include "helper.h"

void readOff(std::string const& filename, std::vector<Point>* points)
{
    points->clear();

   std::fstream infile(filename);
    if (!infile.is_open()) {
        std::cerr << "Error: Unable to open file!" << std::endl;
        return;
    }
    // read header and access the amount of points within the file
    std::string line;
    int num_points;
    // Read the header
    std::getline(infile, line); // Read "OFF" or other header information
    infile >> num_points; // Read the number of points
    std::cout << "number of points to be read = "  << num_points << std::endl;
    std::getline(infile, line); // Read the rest of the line

    // Read the data
    points->reserve(num_points);
    while (num_points > 0) {
        std::getline(infile, line);
        std::istringstream iss(line);
        Point p;
        if (!(iss >> p[0] >> p[1] >> p[2])) {
            std::cerr << "Error: Invalid data format!" << std::endl;
            return;
        }
        points->push_back(p); //TODO: already insert in Spatial Data Structure
        num_points--;
    }

    // Close the file
    infile.close();
}

void writeRuntimeAnalyticsToFile(const runtimeAnalytics* data, const std::string& filename) {
    std::ofstream outFile(filename);

    if (!outFile.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // Write header
    outFile << "numberPoints numberPointsWithinRadius runtimeOctree runtimeBruteForce radius\n";

    // Get the size of the largest vector
    size_t maxSize = std::max({data->numberPoints.size(), data->numberPointsWithinRadius.size(), 
                                data->runtimeOctree.size(), data->runtimeBruteForce.size(),
                                data->radius.size()});

    // Write data
    for (size_t i = 0; i < maxSize; ++i) {
        // Write numberPoints
        if (i < data->numberPoints.size()) {
            outFile << data->numberPoints[i] << " ";
        } else {
            outFile << "0 ";
        }

        // Write numberPointsWithinRadius
        if (i < data->numberPointsWithinRadius.size()) {
            outFile << data->numberPointsWithinRadius[i] << " ";
        } else {
            outFile << "0 ";
        }

        // Write runtimeOctree
        if (i < data->runtimeOctree.size()) {
            outFile << data->runtimeOctree[i].count() << " ";
        } else {
            outFile << "0 ";
        }

        // Write runtimeBruteForce
        if (i < data->runtimeBruteForce.size()) {
            outFile << data->runtimeBruteForce[i].count() << " ";
        } else {
            outFile << "0\n";
        }

        // Write radius Size
        if (i < data->radius.size()) {
            outFile << data->radius[i] << "\n";
        } else {
            outFile << "0\n";
        }
    }

    outFile.close();

    if (!outFile.good()) {
        std::cerr << "Error occurred while writing to file: " << filename << std::endl;
    } else {
        std::cout << "Data successfully written to file: " << filename << std::endl;
    }
}
