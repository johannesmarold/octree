#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/curve_network.h"
#include "polyscope/pick.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <iostream>
#include <string>
#include <utility>
#include <vector>
#include <tuple>

#include "args/args.hxx"
#include "portable-file-dialogs.h"


#include "helper.h"

using Point = std::array<float, 3>; // Eigen Matrix3 for Points
using Cell = std::array<Point , 8>;

using PointList = std::vector<Point>;
using ColorArray = std::vector<glm::vec3>;

int recursionDepth = 4;
int maxLeafContent = 10;
int bbCounter = 0;
bool bbActive = false;

float selectedRadius = 1;

/**
 * struct that builds an element of the octree
 */
struct node
{
    polyscope::CurveNetwork* bb;
    Cell cell;
    std::array <node*, 8> children;
    PointList points;
    bool leaf = true;
    int depth;
    std::vector<int> indices;

    node() : bb(nullptr), leaf(true), depth(0) {
        children.fill(nullptr), cell.fill(Point());
    }
};

runtimeAnalytics* rta = new runtimeAnalytics();

float SnapValue(float value, float step) {
    return roundf(value / step) * step;
}

/**
 * Teach polyscope how to handle our datatype
 */
float adaptorF_custom_accessVector3Value(const Point& v, unsigned int ind)
{
    return v[ind];
}

/**
 * helper function to get the minimum value of a cell (defined as the lowest x, y and z value or lower left front corner)
 *
 * @param c vertices defining the cell
 * @return Point defining the minimum value
 */
Point getMinimumOfCell(Cell c) {
    return c[0];
}

/**
 * helper function to get the maximum value of a cell (defined as the highest x, y and z value or upper right back corner)
 *
 * @param c vertices defining the cell
 * @return Point defining the maximum value
 */
Point getMaximumOfCell(Cell c) {
    return c[7];
}

/**
 * helper function to get the mid Point of a Cell
 *
 * @param n node to get the mid Point from
 * @return Point defining the mid value
 */
Point getMidOfCell(Cell c) {
    Point bbMid;
    bbMid[0] = getMinimumOfCell(c)[0] + (getMaximumOfCell(c)[0] - getMinimumOfCell(c)[0]) / 2.0;
    bbMid[1] = getMinimumOfCell(c)[1] + (getMaximumOfCell(c)[1] - getMinimumOfCell(c)[1]) / 2.0;
    bbMid[2] = getMinimumOfCell(c)[2] + (getMaximumOfCell(c)[2] - getMinimumOfCell(c)[2]) / 2.0;
    return bbMid;
}

/**
 * helper function to build the Cell of the given extend defined by the largest diagonal
 *
 * @param lowerCorner containing minima of x, y, z
 * @param upperCorner containing maxima of x, y, z
 * @return the built Cell
 */
Cell buildCell(Point lowerCorner, Point upperCorner) {
    Point p0 = {lowerCorner[0], lowerCorner[1], lowerCorner[2]}; // lower front left corner
    Point p1 = {upperCorner[0], lowerCorner[1], lowerCorner[2]}; // lower front right corner
    Point p2 = {lowerCorner[0], upperCorner[1], lowerCorner[2]}; // upper front left corner
    Point p3 = {upperCorner[0], upperCorner[1], lowerCorner[2]}; // upper front right corner
    Point p4 = {lowerCorner[0], lowerCorner[1], upperCorner[2]}; // lower back left corner
    Point p5 = {upperCorner[0], lowerCorner[1], upperCorner[2]}; // lower back right corner
    Point p6 = {lowerCorner[0], upperCorner[1], upperCorner[2]}; // upper back left corner
    Point p7 = {upperCorner[0], upperCorner[1], upperCorner[2]}; // upper back right corner

    Cell c = {p0, p1, p2, p3, p4, p5, p6, p7};
    return c;
}

/**
 * helper function to build the bounding Box of the given extend defined by the largest diagonal
 * 
 * @param lowerCorner containing minima of x, y, z
 * @param upperCorner containing maxima of x, y, z
 * @return the built bounding Box as a polyscope CurveNetwork
 */
polyscope::CurveNetwork* buildBB(Point lowerCorner, Point upperCorner) 
{
    std::vector<std::array<float, 3>> vertices = {
        {lowerCorner[0], lowerCorner[1], lowerCorner[2]}, // 0 lower Corner
        {upperCorner[0], lowerCorner[1], lowerCorner[2]}, // 1
        {upperCorner[0], lowerCorner[1], upperCorner[2]}, // 2
        {lowerCorner[0], lowerCorner[1], upperCorner[2]}, // 3
        {lowerCorner[0], upperCorner[1], lowerCorner[2]}, // 4
        {upperCorner[0], upperCorner[1], lowerCorner[2]}, // 5
        {upperCorner[0], upperCorner[1], upperCorner[2]}, // 6 upper Corner
        {lowerCorner[0], upperCorner[1], upperCorner[2]}  // 7
    };

    // Define the edges of the bounding box
    std::vector<std::array<size_t, 2>> edges = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, // bottom face
        {4, 5}, {5, 6}, {6, 7}, {7, 4}, // top face
        {0, 4}, {1, 5}, {2, 6}, {3, 7}  // vertical edges
    };

    std::vector<std::array<size_t, 4>> faceIndices = {
        {0, 1, 2, 3}, // bottom face
        {4, 5, 6, 7}, // top face
        {0, 3, 7, 4}, // left face
        {1, 2, 6, 5}, // right face
        {0, 1, 5, 4}, // front face
        {3, 2, 6, 7} // back face
    };

    polyscope::CurveNetwork* bb = polyscope::registerCurveNetwork("Bounding Box for " + std::to_string(bbCounter), vertices, edges);
    bb->setRadius(0.0005);
    bb->setColor({1, 0, 0});

    bbCounter ++;
    return bb;
}

/**
 * helper function to build first bounding box/Cell calculated according to the given point cloud
 * 
 * @param points point cloud
 * @return root node of the octree
 */
node* minimumBoundingBox(PointList const& points) 
{
    auto xExtrema = std::minmax_element(points.begin(), points.end(),
                                     [](const Point& lhs, const Point& rhs) {
                                        return lhs[0] < rhs[0];
                                     });
    auto yExtrema = std::minmax_element(points.begin(), points.end(),
                                     [](const Point& lhs, const Point& rhs) {
                                        return lhs[1] < rhs[1];
                                     });

    auto zExtrema = std::minmax_element(points.begin(), points.end(),
                                     [](const Point& lhs, const Point& rhs) {
                                        return lhs[2] < rhs[2];
                                     });

    Point lowerCorner = {(*xExtrema.first)[0], (*yExtrema.first)[1], (*zExtrema.first)[2]};
    Point upperCorner = {(*xExtrema.second)[0], (*yExtrema.second)[1], (*zExtrema.second)[2]};

    node* root = new node;

    if (bbActive) {
        root->bb = buildBB(lowerCorner, upperCorner);
    }
    root->cell = buildCell(lowerCorner, upperCorner);

    return root;
}

/**
 * helper function to determine to which sub cell of the 8 sub cells of a cell
 * the given point is assigned to.
 *
 * @param p point from the point cloud
 * @param n node, in which the point is located
 * @return determines the index for the sub cell
 */
int comparePointToMidCell(Point const& p, node* n) {
    int i = 0;

    // calculate mid of bounding box 
    Point bbMid = getMidOfCell(n->cell);

    if (p[0] > bbMid[0]){i = i+1;}
    if (p[1] > bbMid[1]){i = i+2;}
    if (p[2] > bbMid[2]){i = i+4;}

    return i;
}

/**
 * helper function to build the cells of all 8 sub cells of a given node
 *
 * @param l given parent node, where we build the cells for all 8 children
 */
void buildSubDivisionCells(node* l)
{
    Point bbMid = getMidOfCell(l->cell);
    Point low = getMinimumOfCell(l->cell);
    Point high = getMaximumOfCell(l->cell);

    // division 0
    l->children[0]->cell = buildCell(low, bbMid);
    // division 1
    l->children[1]->cell = buildCell({bbMid[0], low[1], low[2]}, {high[0], bbMid[1], bbMid[2]});
    // division 2
    l->children[2]->cell = buildCell({low[0], bbMid[1], low[2]}, {bbMid[0], high[1], bbMid[2]});
    // division 3
    l->children[3]->cell = buildCell({bbMid[0], bbMid[1], low[2]}, {high[0], high[1], bbMid[2]});
    // division 4
    l->children[4]->cell = buildCell({low[0], low[1], bbMid[2]}, {bbMid[0], bbMid[1], high[2]});
    // division 5
    l->children[5]->cell = buildCell({bbMid[0], low[1], bbMid[2]}, {high[0], bbMid[1], high[2]});
    // division 6
    l->children[6]->cell = buildCell({low[0], bbMid[1], bbMid[2]}, {bbMid[0], high[1], high[2]});
    // division 7
    l->children[7]->cell = buildCell(bbMid, high);
}

/**
 * helper function to build the bounding boxes of all 8 sub cells of a given node
 *
 * @param l given parent node, where we build the bounding boxes for all 8 children 
 */
void buildSubDivisionBBs(node* l)
{
    Point bbMid = getMidOfCell(l->cell);
    Point low = getMinimumOfCell(l->cell);
    Point high = getMaximumOfCell(l->cell);

    // division 0
    l->children[0]->bb = buildBB(low, bbMid);
    // division 1
    l->children[1]->bb = buildBB({bbMid[0], low[1], low[2]}, {high[0], bbMid[1], bbMid[2]});
    // division 2
    l->children[2]->bb = buildBB({low[0], bbMid[1], low[2]}, {bbMid[0], high[1], bbMid[2]});
    // division 3
    l->children[3]->bb = buildBB({bbMid[0], bbMid[1], low[2]}, {high[0], high[1], bbMid[2]});
    // division 4
    l->children[4]->bb = buildBB({low[0], low[1], bbMid[2]}, {bbMid[0], bbMid[1], high[2]});
    // division 5
    l->children[5]->bb = buildBB({bbMid[0], low[1], bbMid[2]}, {high[0], bbMid[1], high[2]});
    // division 6
    l->children[6]->bb = buildBB({low[0], bbMid[1], bbMid[2]}, {bbMid[0], high[1], high[2]});
    // division 7
    l->children[7]->bb = buildBB(bbMid, high);
}

/**
 * helper function to split a node and determine which points are allocated to which child
 * 
 * @param l parent node
 * @param depth depth of the children nodes
 */
void splitLeaf(node* l, int depth) {

    l->leaf = false;
    
    for (node* &child : l->children)
    {
        child = new node();
        child->depth = depth;
    }

    // Build bounding boxes for the children
    if (bbActive) {
        buildSubDivisionBBs(l);
    }
    buildSubDivisionCells(l);

    for (size_t j = 0; j < l->points.size(); j++) 
    {
        int i = comparePointToMidCell(l->points[j], l);
        l->children[i]->points.push_back(l->points[j]);
        l->children[i]->indices.push_back(l->indices[j]);
    }
    l->points.clear();
    l->indices.clear();
}

/**
 * helper function to insert a point from the point cloud into the octree
 *
 * @param p pointer to point from point cloud
 * @param index of p at Point Cloud Array
 * @param root root node of the octree
 */
void insertPointInOctree(Point const& p, int index, node* root) {
    node* n = root;
    int depthCounter = 1;

    while (!n->leaf && n->depth < recursionDepth)
    {
        int i = comparePointToMidCell(p, n);
        n = n->children[i];
        depthCounter++;
    }

    if (int(n->points.size()) >= maxLeafContent && n->depth < recursionDepth) {
        n->points.push_back(p);
        n->indices.push_back(index);
        splitLeaf(n, depthCounter);
    }
    else {
        n->points.push_back(p);
        n->indices.push_back(index);
    }
}

/**
 * function to build the spatial data structure octree for a point cloud
 *
 * @param points point cloud
 * @return root node of octree
 */
node* buildOctree(PointList const& points)
{
    node* root = minimumBoundingBox(points);

    for (size_t i = 0; i < points.size(); i++)
     {
        insertPointInOctree(points[i], i, root);
    }
    
    std::cout << "finished building octree" << std::endl;
    return root;
}

/**
 * struct for calculating the Euclidian Distance of two Points to each other
 */
struct EuclideanDistance
{
    static float measure(Point const& p1, Point const& p2)
    {
        float dx = p1[0] - p2[0];
        float dy = p1[1] - p2[1];
        float dz = p1[2] - p2[2];
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }
};

/**
 * struct for estimating the Euclidian Distance from a Point to a cell
 * method: simply calculate the maximum distance from the Point to the cell
 * note: this underestimates the distance constantly, but is easier to compute since
 *       you don't have to compute the square root. The correct distance is finally
 *       computed by Euclidian Distance of the selected Points
 */
struct SimpleDistance
{
    static float measure(Point const& p1, node* cell) {
        float d = 0;
        for (size_t i = 0; i < 3; i++) {
            float t = p1[i] - getMaximumOfCell(cell->cell)[i];
            if (t > d){d = t;}
            t = getMinimumOfCell(cell->cell)[i] - p1[i];
            if (t > d){d = t;}
        }
        return d;
    }
};

/**
 * helper function to delete the current octree before loading a new OFF file
 *
 * @param root of the octree
 */
void deleteOctree(node* root) {
    if (root == nullptr) return;
    // Recursively delete children
    for (node* child : root->children) {
        deleteOctree(child);
    }

    // Deregister the bounding box if it's not null
    if (root->bb != nullptr) {
        polyscope::removeStructure(root->bb);
    }

    // Delete the node
    delete root;
}

/**
 * function for the octree to check, which Points of a Point Cloud lay within
 * the radius of a certain Point
 *
 * @param root of the octree
 * @param p selected point for which points within the radius are searched for
 * @param radius of the search
 * @param result list of the Points, which lay within the radius
 */
void checkInRadius(node* root, Point p, float radius, std::vector<std::size_t>* result) {
    if (root == nullptr) {return;}
    // check for all points in leaf, if they are within the radius
    if (root->leaf) {
        for (size_t i = 0; i < root->points.size(); i++) {
            if (EuclideanDistance::measure(root->points[i], p) <= radius) {
                result->push_back(root->indices[i]);
            }
        }
        return;
    }
    // check for all cells, if they intersect with the radius
    for(node* &child : root->children) {
        if (SimpleDistance::measure(p, child) < radius) {
            checkInRadius(child, p, radius, result);
        }
    }
}

/**
 * Implementation of a Spatial Data Structure Class
 *
 * class members are the original Point Cloud and the built Octree
 */
class SpatialDataStructure
{
public:
    SpatialDataStructure(PointList const& points)
        : m_points(points),
        octree(buildOctree(points))
    {
    }

    virtual ~SpatialDataStructure() {
        deleteOctree(octree);
    }

    PointList const& getPoints() const
    {
        return m_points;
    }

    node* const& getOctree() const
    {
        return octree;
    }

    /**
     * method collects Points within the radius of a certain Point with Brute-Force
     *
     * @param p point for which the radius is searched
     * @param radius of the search
     * @return Pair consisting of the amount of Points within the radius and the computation time
     */
    virtual std::tuple<int, std::chrono::duration<double>> collectInRadiusBruteForce(const Point& p, float radius) const
    {
        std::vector<std::size_t> result;

        // Dummy brute-force implementation
        auto start = std::chrono::high_resolution_clock::now();
        for (std::size_t i = 0; i < m_points.size(); ++i)
        {
            float distance = EuclideanDistance::measure(p, m_points[i]);
            if (distance <= radius) {
                result.push_back(i);
            }
        }
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        std::cout << "Execution time brute force: " << duration.count() << " seconds" << std::endl;

        // Visualize the points within the radius in green
        if (!result.empty()) {
            std::cout << "result amount brute force: " << result.size() << std::endl;
            glm::vec3 currentColor = polyscope::getPointCloud("Points")->getPointColor();
            // Initialize the vector with the given size and color
            ColorArray newColor(m_points.size(), currentColor);

            for (size_t i = 0; i < result.size(); i++){
                newColor[result[i]] = {0.0, 1.0, 0.0};
            }
            polyscope::getPointCloud("Points")->addColorQuantity("radius colored Brute Force", newColor);
        }
    
        return std::make_tuple(result.size(), duration);
    }

    /**
     * method collects Points within the radius of a certain Point by using the octree
     *
     * @param p point for which the radius is searched
     * @param radius of the search
     * @return Pair consisting of the amount of Points within the radius and the computation time
     */
    virtual std::tuple<int, std::chrono::duration<double>> collectInRadius(const Point& p, float radius) const
    {
        std::vector<std::size_t> result;

        auto start = std::chrono::high_resolution_clock::now();
        checkInRadius(octree, p, radius, &result);
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> duration = end - start;
        std::cout << "Execution time octree: " << duration.count() << " seconds" << std::endl;
        std::cout << "points within radius amount: " << result.size() <<std::endl;

        // Visualize the points within the radius in green
        if (!result.empty()) {
            std::cout << "result amount: " << result.size() << std::endl;
            glm::vec3 currentColor = polyscope::getPointCloud("Points")->getPointColor();
            // Initialize the vector with the given size and color
            ColorArray newColor(m_points.size(), currentColor);

            for (size_t i = 0; i < result.size(); i++){
                newColor[result[i]] = {1.0, 0.0, 0.0};
            }
            polyscope::getPointCloud("Points")->addColorQuantity("radius colored Octree", newColor);
        }
    
        return std::make_tuple(result.size(), duration);
    }

    /**
     * method to delete the octree
     *
     * @param root of the octree
     */
    virtual void deleteOctree(node* root)
    {
        ::deleteOctree(root);
        bbCounter = 0;
    }

private:
    PointList m_points;
    node* octree;
};

// Application variables
polyscope::PointCloud* pc = nullptr;
std::unique_ptr<SpatialDataStructure> sds;

void callback() {
    if (ImGui::Button("Load Off")) {
        auto paths = pfd::open_file("Load Off", "", std::vector<std::string>{ "point data (*.off)", "*.off" }, pfd::opt::none).result();
        if (!paths.empty())
        {
            std::filesystem::path path(paths[0]);

            if (path.extension() == ".off")
            {
                if(sds) {
                    sds.reset();
                }
                // Read the point cloud
                std::vector<Point> points;
                std::cout << "pathString: " << std::filesystem::current_path() << std::endl;
                readOff(path.string(), &points);

                // Create the polyscope geometry
                pc = polyscope::registerPointCloud("Points", points);
                pc->setEnabled(true);

                // Build spatial data structure
                sds = std::make_unique<SpatialDataStructure>(points);
            }
        }
    }

    if (ImGui::Button("Radius Search")) {

        std::__1::pair<polyscope::Structure *, size_t> selected = polyscope::pick::getSelection();
        if (selected.first == nullptr) {
            std::cout << "no point is selected" << std::endl;
        } else {
            rta->numberPoints.push_back(sds->getPoints().size());
            auto [radiusSize, duration] = sds->collectInRadius(sds->getPoints()[selected.second], selectedRadius);
            rta->numberPointsWithinRadius.push_back(radiusSize);
            rta->runtimeOctree.push_back(duration);
            duration = std::get<1>(sds->collectInRadiusBruteForce(sds->getPoints()[selected.second], selectedRadius));
            rta->runtimeBruteForce.push_back(duration);
            rta->radius.push_back(selectedRadius);
        }
    }

    if (ImGui::SliderFloat("Radius", &selectedRadius, 0.0f, 2.0f)) {
        selectedRadius = SnapValue(selectedRadius, 0.1);
    }
    //ImGui::SliderFloat("Radius", &selectedRadius, 0.0, 1.0, 0.1);

    ImGui::SliderInt("Max points per Cell", &maxLeafContent, 1, 50);

    if (ImGui::Button("Write Analytics")) {
        writeRuntimeAnalyticsToFile(rta, "../../runtimeAnalytics.txt");
    }

    ImGui::SliderInt("Recursion Depth", &recursionDepth, 1, 10);

    if (ImGui::Button(bbActive ? "Visualize Cells off" : "Visualize Cells on")) {
        bbActive = !bbActive; // Toggle the boolean value
    }
}

int main(int argc, char** argv) {
  // Configure the argument parser
  args::ArgumentParser parser("Computer Graphics Seminar Spatial Data Structure.");

  // Parse args
  try {
    parser.ParseCLI(argc, argv);
  } catch (const args::Help&) {
    std::cout << parser;
    return 0;
  } catch (const args::ParseError& e) {
    std::cerr << e.what() << std::endl;

    std::cerr << parser;
    return 1;
  }

  // Options
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
  polyscope::options::shadowBlurIters = 6;

  // Initialize polyscope
  polyscope::init();

  // Add a few gui elements
  polyscope::state::userCallback = callback;

  // Show the gui
  polyscope::show();

  return 0;
}
