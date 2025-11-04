#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#include <ctime>
#include <opencv2/opencv.hpp>

struct Point {
    float x, y;
    Point(float x = 0, float y = 0) : x(x), y(y) {}
};

struct LineSegment {
    Point start, end;
    LineSegment(Point s, Point e) : start(s), end(e) {}
};

class MazeGenerator {
private:
    int width, height;
    std::vector<std::vector<bool>> visited;
    std::vector<LineSegment> walls;
    std::mt19937 rng;

    // Directions: North, East, South, West
    const int dx[4] = { 0, 1, 0, -1 };
    const int dy[4] = { -1, 0, 1, 0 };

    bool isValid(int x, int y) {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

public:
    MazeGenerator(int w, int h, unsigned seed = 0) : width(w), height(h) {
        if (seed == 0) {
            seed = static_cast<unsigned>(time(nullptr));
        }
        rng.seed(seed);
        visited.resize(height, std::vector<bool>(width, false));
    }

    void generate() {
        walls.clear();
        visited.assign(height, std::vector<bool>(width, false));

        std::vector<std::vector<bool>> rightWall(height, std::vector<bool>(width, true));
        std::vector<std::vector<bool>> bottomWall(height, std::vector<bool>(width, true));

        carvePassagesWithWalls(0, 0, rightWall, bottomWall);

        // Convert grid to line segments
        // Each cell is 1x1 unit, starting at (0,0)
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // Right wall
                if (rightWall[y][x] && x < width - 1) {
                    walls.push_back(LineSegment(
                        Point(x + 1, y),
                        Point(x + 1, y + 1)
                    ));
                }
                // Bottom wall
                if (bottomWall[y][x] && y < height - 1) {
                    walls.push_back(LineSegment(
                        Point(x, y + 1),
                        Point(x + 1, y + 1)
                    ));
                }
            }
        }

        // Add outer boundary walls
        // Top wall
        for (int x = 0; x < width; x++) {
            walls.push_back(LineSegment(Point(x, 0), Point(x + 1, 0)));
        }
        // Left wall
        for (int y = 0; y < height; y++) {
            walls.push_back(LineSegment(Point(0, y), Point(0, y + 1)));
        }
        // Right outer wall
        for (int y = 0; y < height; y++) {
            walls.push_back(LineSegment(Point(width, y), Point(width, y + 1)));
        }
        // Bottom outer wall
        for (int x = 0; x < width; x++) {
            walls.push_back(LineSegment(Point(x, height), Point(x + 1, height)));
        }
    }

    void carvePassagesWithWalls(int x, int y,
        std::vector<std::vector<bool>>& rightWall,
        std::vector<std::vector<bool>>& bottomWall) {
        visited[y][x] = true;

        std::vector<int> directions = { 0, 1, 2, 3 };
        std::shuffle(directions.begin(), directions.end(), rng);

        for (int dir : directions) {
            int nx = x + dx[dir];
            int ny = y + dy[dir];

            if (isValid(nx, ny) && !visited[ny][nx]) {
                // Remove wall between current and next cell
                if (dir == 0) { // North
                    bottomWall[ny][nx] = false;
                }
                else if (dir == 1) { // East
                    rightWall[y][x] = false;
                }
                else if (dir == 2) { // South
                    bottomWall[y][x] = false;
                }
                else { // West
                    rightWall[ny][nx] = false;
                }

                carvePassagesWithWalls(nx, ny, rightWall, bottomWall);
            }
        }
    }

    const std::vector<LineSegment>& getWalls() const {
        return walls;
    }

    cv::Mat renderToImage(int cellSize, int wallThickness) const {
        // Calculate image size
        int imgWidth = width * cellSize + wallThickness;
        int imgHeight = height * cellSize + wallThickness;

        // Create black image
        cv::Mat image(imgHeight, imgWidth, CV_8UC3, cv::Scalar(0, 0, 0));

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

        return image;
    }

    void printMaze() const {
        // Print top border
        for (int x = 0; x < width; x++) {
            std::cout << "+---";
        }
        std::cout << "+\n";

        // Print maze (simple ASCII visualization)
        for (int y = 0; y < height; y++) {
            std::cout << "|";
            for (int x = 0; x < width; x++) {
                std::cout << "   ";
                // Check if there's a right wall
                bool hasRightWall = false;
                for (const auto& wall : walls) {
                    if (wall.start.x == x + 1 && wall.start.y == y &&
                        wall.end.x == x + 1 && wall.end.y == y + 1) {
                        hasRightWall = true;
                        break;
                    }
                }
                std::cout << (hasRightWall ? "|" : " ");
            }
            std::cout << "\n+";

            // Bottom walls
            for (int x = 0; x < width; x++) {
                bool hasBottomWall = false;
                for (const auto& wall : walls) {
                    if (wall.start.x == x && wall.start.y == y + 1 &&
                        wall.end.x == x + 1 && wall.end.y == y + 1) {
                        hasBottomWall = true;
                        break;
                    }
                }
                std::cout << (hasBottomWall ? "---" : "   ");
                std::cout << "+";
            }
            std::cout << "\n";
        }
    }
};

int main() {
    int width, height, cellSize, wallThickness;

    std::cout << "=== Maze Generator with OpenCV ===" << std::endl;
    std::cout << "\nEnter maze width (X cells): ";
    std::cin >> width;
    std::cout << "Enter maze height (Y cells): ";
    std::cin >> height;
    std::cout << "Enter cell size (pixels): ";
    std::cin >> cellSize;
    std::cout << "Enter wall thickness (pixels): ";
    std::cin >> wallThickness;

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
    cv::imshow("Maze", mazeImage);

    std::cout << "\nPress any key in the image window to exit..." << std::endl;
    cv::waitKey(0);

    // Optional: Print ASCII version
    std::cout << "\nASCII Visualization:" << std::endl;
    maze.printMaze();

    std::cout << "\nTotal wall segments: " << maze.getWalls().size() << std::endl;

    return 0;
}