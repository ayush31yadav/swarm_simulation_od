#pragma once
#include "mazeGeneration.hpp"
#include "graph.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>

cv::Mat getVidImage(MazeGenerator maze, std::vector<Point> pointCloud, std::vector<Point> drones, int cellSize, int wallThickness, int midBorder) {
    std::vector<LineSegment> walls = maze.getWalls();
    int width = maze.getWidth();
    int height = maze.getHeight();

    // Calculate image size
    int imgWidth = width * cellSize + wallThickness;
    int imgHeight = height * cellSize + wallThickness;

    // Create black image
    cv::Mat image(imgHeight, imgWidth * 2, CV_8UC3, cv::Scalar(0, 0, 0));

    // Draw walls in white
    for (const auto& wall : walls) {
        cv::Point p1(
            static_cast<int>(wall.start.x * cellSize),
            static_cast<int>(wall.start.y * cellSize)
        );
        cv::Point p2(
            static_cast<int>(wall.end.x * cellSize),
            static_cast<int>(wall.end.y * cellSize)
        );

        cv::line(image, p1, p2, cv::Scalar(255, 255, 255), wallThickness);
    }

    for (Point drone : drones) {
        cv::Point tL1(drone.x * cellSize - 2, drone.y * cellSize - 2);
        cv::Point bR1(drone.x * cellSize + 2, drone.y * cellSize + 2);

        cv::Point tL2((drone.x + width) * cellSize - 2 + midBorder, drone.y * cellSize - 2);
        cv::Point bR2((drone.x + width) * cellSize + 2 + midBorder, drone.y * cellSize + 2);

        cv::rectangle(image, tL2, bR2, cv::Scalar(0, 255, 0));
        cv::rectangle(image, tL1, bR1, cv::Scalar(0, 255, 0));
    }

    for (Point p : pointCloud) {
        cv::Point PC((p.x + width) * cellSize + midBorder, p.y * cellSize);
        
        cv::circle(image, PC, 1, cv::Scalar(0, 0, 255), 1);
    }

    return image;
}

bool rayIntersectSegment(const Point& rayStart, const Point& rayDir, const LineSegment& seg, Point& intersection) {
    float x1 = rayStart.x, y1 = rayStart.y;
    float x2 = rayStart.x + rayDir.x, y2 = rayStart.y + rayDir.y;
    float x3 = seg.start.x, y3 = seg.start.y;
    float x4 = seg.end.x, y4 = seg.end.y;

    float denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    if (std::abs(denom) < 1e-6) return false; // Parallel or coincident

    float t1 = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
    float t2 = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

    // t1: position along ray (>= 0). t2: position along segment [0,1]
    if (t1 >= 0 && t2 >= 0 && t2 <= 1) {
        intersection.x = x1 + t1 * (x2 - x1);
        intersection.y = y1 + t1 * (y2 - y1);
        return true;
    }
    return false;
}

// Main intersection search
Point getPointOfIntersection(MazeGenerator maze, Point srcPoint, float angle) {
    std::vector<LineSegment> walls = maze.getWalls();
    // Ray direction vector
    float rayLength = 50.0f; // sufficiently long
    float dx = std::cos(angle) * rayLength;
    float dy = -std::sin(angle) * rayLength; // invert Y for image coordinates
    Point rayDir(dx, dy);

    Point nearestIntersection;
    float minDist = std::numeric_limits<float>::max();
    bool found = false;

    for (const auto& seg : walls) {
        Point intersection;
        if (rayIntersectSegment(srcPoint, rayDir, seg, intersection)) {
            float dist = std::hypot(intersection.x - srcPoint.x, intersection.y - srcPoint.y);
            if (dist < minDist) {
                minDist = dist;
                nearestIntersection = intersection;
                found = true;
            }
        }
    }
    // If nothing found, return srcPoint (or use std::optional / nullptr, as needed)
    return found ? nearestIntersection : srcPoint;
}

std::vector<Point> scanAt(MazeGenerator maze, Point location, float minLen, int scanPoints = 36) {

    std::vector<Point> out;

    for (int i = 0; i < scanPoints; i++) {
        float angle = 2.0f * CV_PI * i / scanPoints;

        Point p = getPointOfIntersection(maze, location, angle);

        if (!((p.x == location.x) && (p.y == location.y))) {
            if (std::pow(location.x - p.x, 2) + std::pow(location.y - p.y, 2) >= std::pow(minLen, 2)) {
                out.push_back(p);
            }
        }
    }

    return out;
}

// Generate free-space samples from scans (walkable area)
std::vector<Point> getFreeSpaceSamples(const Point& drone, const std::vector<Point>& hits, int samplesPerRay = 5) {
    std::vector<Point> freePoints;
    for (const Point& hit : hits) {
        float dx = hit.x - drone.x;
        float dy = hit.y - drone.y;
        for (int s = 1; s < samplesPerRay; ++s) {
            float t = static_cast<float>(s) / samplesPerRay;
            freePoints.emplace_back(drone.x + dx * t, drone.y + dy * t);
        }
    }
    return freePoints;
}

// Choose next move direction based on open space
Point pickNextDirection(const Point& drone, const std::vector<Point>& freePoints, float stepLen) {
    if (freePoints.empty()) return drone;

    // Average free-space vector to get dominant open direction
    float avgX = 0, avgY = 0;
    for (const auto& p : freePoints) {
        avgX += (p.x - drone.x);
        avgY += (p.y - drone.y);
    }
    avgX /= freePoints.size();
    avgY /= freePoints.size();

    // Normalize and move a step forward
    float len = std::sqrt(avgX * avgX + avgY * avgY);
    if (len < 1e-4) return drone;
    avgX /= len;
    avgY /= len;

    return Point(drone.x + avgX * stepLen, drone.y + avgY * stepLen);
}


// Add these helper functions before main()
float distance(const Point& a, const Point& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

// Check if a point is too close to any wall
bool isTooCloseToWall(const Point& p, const std::vector<LineSegment>& walls, float minDist) {
    for (const auto& wall : walls) {
        // Calculate distance from point to line segment
        float x1 = wall.start.x, y1 = wall.start.y;
        float x2 = wall.end.x, y2 = wall.end.y;
        float px = p.x, py = p.y;

        // Vector from start to end
        float dx = x2 - x1;
        float dy = y2 - y1;
        float lenSq = dx * dx + dy * dy;

        if (lenSq < 1e-6) {
            // Wall is a point, check distance to point
            if (distance(p, wall.start) < minDist) return true;
            continue;
        }

        // Project point onto line segment
        float t = ((px - x1) * dx + (py - y1) * dy) / lenSq;
        t = std::max(0.0f, std::min(1.0f, t)); // Clamp to [0,1]

        // Find closest point on segment
        float closestX = x1 + t * dx;
        float closestY = y1 + t * dy;

        // Check distance
        float dist = std::sqrt(std::pow(px - closestX, 2) + std::pow(py - closestY, 2));
        if (dist < minDist) return true;
    }
    return false;
}

// Check if path between two points crosses too close to walls
bool isPathSafe(const Point& start, const Point& end, const std::vector<LineSegment>& walls, float minDist, int numChecks = 10) {
    for (int i = 0; i <= numChecks; i++) {
        float t = static_cast<float>(i) / numChecks;
        Point checkPoint(
            start.x + t * (end.x - start.x),
            start.y + t * (end.y - start.y)
        );
        if (isTooCloseToWall(checkPoint, walls, minDist)) return false;
    }
    return true;
}

// Find nearest unexplored point to a given location
Point findNearestUnexplored(const Point& loc, const std::vector<Point>& unexplored) {
    if (unexplored.empty()) return Point(-1, -1);

    Point nearest = unexplored[0];
    float minDist = distance(loc, nearest);

    for (const Point& p : unexplored) {
        float dist = distance(loc, p);
        if (dist < minDist) {
            minDist = dist;
            nearest = p;
        }
    }
    return nearest;
}

// Check if point already exists in vector (with tolerance)
bool pointExists(const Point& p, const std::vector<Point>& points, float tolerance = 0.05) {
    for (const Point& existing : points) {
        if (distance(p, existing) < tolerance) return true;
    }
    return false;
}

// Replace the initialization and main loop section with this fixed version:

int main() {
    int width, height, cellSize, wallThickness, dN;
    float stepLen = 0.1;

    std::cout << "=== Maze Generator with OpenCV ===" << std::endl;
    std::cout << "\nEnter maze width (X cells): ";
    std::cin >> width;
    std::cout << "Enter maze height (Y cells): ";
    std::cin >> height;
    std::cout << "Enter cell size (pixels): ";
    std::cin >> cellSize;
    std::cout << "Enter wall thickness (pixels): ";
    std::cin >> wallThickness;
    std::cout << "Enter drone count: ";
    std::cin >> dN;

    MazeGenerator maze(width, height);
    maze.generate();

    std::cout << "\nGenerating maze image..." << std::endl;

    // Render maze to image
    cv::Mat mazeImage = maze.renderToImage(cellSize, wallThickness);

    // Save the image
    std::string filename = "maze_output.png";
    cv::imwrite(filename, mazeImage);
    std::cout << "Maze saved as: " << filename << std::endl;
    std::cout << "Image size: " << mazeImage.cols << "x" << mazeImage.rows << " pixels" << std::endl;

    // Display the maze
    cv::namedWindow("Maze", cv::WINDOW_AUTOSIZE);

    std::vector<Point> pointCloud;
    std::vector<Point> unexplored;
    std::vector<Point> droneLoc;
    std::vector<bool> toScan;
    std::vector<bool> isGoingToLoc;
    std::vector<std::vector<Node>> paths;

    // FIXED: Create graph first
    Graph g(stepLen);

    // FIXED: Properly allocate initial node
    Node* initial = new Node(0.5, 0.5);
    g.addInitialNode(initial);

    // init drones
    std::vector<Node> empty;
    for (int i = 0; i < dN; i++) {
        droneLoc.push_back(Point(0.5, 0.5));
        toScan.push_back(true);
        isGoingToLoc.push_back(false);
        paths.push_back(empty);
    }

    bool goalReached = false;
    Point goalPoint(width - 0.5, height - 0.5);
    std::vector<Point> targetPoints(dN, Point(-1, -1));
    std::vector<LineSegment> walls = maze.getWalls();

    std::cout << "\nStarting simulation..." << std::endl;
    std::cout << "Initial drone position: (0.5, 0.5)" << std::endl;
    std::cout << "Goal position: (" << goalPoint.x << ", " << goalPoint.y << ")" << std::endl;

    int frameCount = 0;

    while (true) {
        frameCount++;

        // Debug output every 100 frames
        if (frameCount % 100 == 0) {
            std::cout << "Frame " << frameCount
                << " - Point cloud size: " << pointCloud.size()
                << ", Unexplored: " << unexplored.size()
                << ", Graph nodes: " << g.getNodes().size() << std::endl;
            for (int i = 0; i < dN; i++) {
                std::cout << "  Drone " << i << ": (" << droneLoc[i].x << ", " << droneLoc[i].y
                    << ") Scan=" << toScan[i] << " GoingToLoc=" << isGoingToLoc[i] << std::endl;
            }
        }

        // Check if any drone reached the goal
        if (!goalReached) {
            for (int i = 0; i < dN; i++) {
                if (distance(droneLoc[i], goalPoint) < stepLen) {
                    goalReached = true;
                    std::cout << "Goal reached by drone " << i << "! All drones converging..." << std::endl;
                    break;
                }
            }
        }

        // If goal reached, make all drones go to goal
        if (goalReached) {
            for (int i = 0; i < dN; i++) {
                if (distance(droneLoc[i], goalPoint) > stepLen * 0.5) {
                    // Find path to goal if not already going there
                    if (targetPoints[i].x != goalPoint.x || targetPoints[i].y != goalPoint.y) {
                        Node* srcNode = nullptr;
                        Node* destNode = nullptr;

                        // Find closest node to current position
                        float minDist = std::numeric_limits<float>::max();
                        for (Node* node : g.getNodes()) {
                            float dist = distance(droneLoc[i], Point(node->x, node->y));
                            if (dist < minDist) {
                                minDist = dist;
                                srcNode = node;
                            }
                        }

                        // Find or create goal node
                        minDist = std::numeric_limits<float>::max();
                        for (Node* node : g.getNodes()) {
                            float dist = distance(goalPoint, Point(node->x, node->y));
                            if (dist < minDist) {
                                minDist = dist;
                                destNode = node;
                            }
                        }

                        if (destNode == nullptr && srcNode != nullptr) {
                            destNode = g.addNode(srcNode, goalPoint.x, goalPoint.y);
                        }

                        if (srcNode && destNode) {
                            std::vector<Node*> path = g.findPath(srcNode, destNode);
                            paths[i].clear();
                            for (Node* n : path) {
                                paths[i].push_back(*n);
                            }
                            targetPoints[i] = goalPoint;
                            isGoingToLoc[i] = true;
                            toScan[i] = false;
                        }
                    }

                    // Move along path
                    if (!paths[i].empty()) {
                        Node target = paths[i][0];
                        Point targetPt(target.x, target.y);
                        float dist = distance(droneLoc[i], targetPt);

                        if (dist < stepLen * 0.5) {
                            if (!paths[i].empty()) paths[i].erase(paths[i].begin());
                            continue; // move to next iteration
                        }
                        else {
                            float dx = (targetPt.x - droneLoc[i].x) / dist;
                            float dy = (targetPt.y - droneLoc[i].y) / dist;
                            Point nextPos(droneLoc[i].x + dx * stepLen, droneLoc[i].y + dy * stepLen);

                            if (!isTooCloseToWall(nextPos, walls, stepLen * 0.8)) {
                                droneLoc[i] = nextPos;
                            }
                            else {
                                paths[i].erase(paths[i].begin());
                            }
                        }
                    }
                }
            }
        }
        else {
            // Normal exploration mode
            for (int i = 0; i < dN; i++) {
                if (toScan[i]) {
                    // Scan walls around current position
                    std::vector<Point> scans = scanAt(maze, droneLoc[i], stepLen, 36);

                    // Derive free-space samples (points between drone and walls)
                    std::vector<Point> freePoints = getFreeSpaceSamples(droneLoc[i], scans, 5);

                    // Add all scanned points to global point cloud for visualization
                    pointCloud.insert(pointCloud.end(), scans.begin(), scans.end());

                    // Pick next movement direction from open space
                    Point next = pickNextDirection(droneLoc[i], freePoints, stepLen);

                    // Move if safe
                    if (!isTooCloseToWall(next, walls, stepLen * 0.4)) {
                        droneLoc[i] = next;
                    }

                    if (frameCount % 50 == 0) {
                        std::cout << "Drone " << i << " moved to ("
                            << droneLoc[i].x << ", " << droneLoc[i].y << ")\n";
                    }
                }
                else {
                    if (isGoingToLoc[i]) {
                        // Check if reached target
                        if (distance(droneLoc[i], targetPoints[i]) < stepLen * 0.5) {
                            toScan[i] = true;
                            isGoingToLoc[i] = false;
                            paths[i].clear();
                            if (frameCount % 100 == 0) {
                                std::cout << "  Drone " << i << " reached target" << std::endl;
                            }
                        }
                        else {
                            // Move along path
                            if (!paths[i].empty()) {
                                Node target = paths[i][0];
                                Point targetPt(target.x, target.y);
                                float dist = distance(droneLoc[i], targetPt);

                                if (dist < stepLen * 0.5) {
                                    paths[i].erase(paths[i].begin());
                                }
                                else {
                                    float dx = (targetPt.x - droneLoc[i].x) / dist;
                                    float dy = (targetPt.y - droneLoc[i].y) / dist;
                                    Point nextPos(droneLoc[i].x + dx * stepLen, droneLoc[i].y + dy * stepLen);

                                    if (!isTooCloseToWall(nextPos, walls, stepLen * 0.7)) {
                                        droneLoc[i] = nextPos;
                                    }
                                    else {
                                        toScan[i] = true;
                                        isGoingToLoc[i] = false;
                                        paths[i].clear();
                                        std::cout << "Drone " << i << " aborted path due to wall collision" << std::endl;
                                    }
                                }
                            }
                            else {
                                toScan[i] = true;
                                isGoingToLoc[i] = false;
                            }
                        }
                    }
                }
            }
        }

        // Display
        cv::Mat frame = getVidImage(maze, pointCloud, droneLoc, cellSize, wallThickness, 50);
        cv::imshow("Maze", frame);

        // Exit on key press
        if (cv::waitKey(30) >= 0) break;

        if (goalReached) {
            bool allReached = true;
            for (int i = 0; i < dN; i++) {
                if (distance(droneLoc[i], goalPoint) > stepLen * 0.5) {
                    allReached = false;
                    break;
                }
            }
            if (allReached) {
                std::cout << "All drones reached the goal!" << std::endl;
                cv::waitKey(3000);
                break;
            }
        }
    }

    std::cout << "\nPress any key in the image window to exit..." << std::endl;
    cv::waitKey(0);

    // Cleanup
    delete initial;

    return 0;
}