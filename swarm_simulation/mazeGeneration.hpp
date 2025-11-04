#ifndef MAZE_GENERATOR_HPP
#define MAZE_GENERATOR_HPP

#include <vector>
#include <random>
#include <opencv2/opencv.hpp>

struct Point {
    float x, y;
    Point(float x = 0, float y = 0);
};

struct LineSegment {
    Point start, end;
    LineSegment(Point s, Point e);
};

class MazeGenerator {
private:
    int width, height;
    std::vector<std::vector<bool>> visited;
    std::vector<LineSegment> walls;
    std::mt19937 rng;

    // Directions: North, East, South, West
    static const int dx[4];
    static const int dy[4];

    bool isValid(int x, int y);
    void carvePassagesWithWalls(int x, int y,
        std::vector<std::vector<bool>>& rightWall,
        std::vector<std::vector<bool>>& bottomWall);

public:
    MazeGenerator(int w, int h, unsigned seed = 0);
    void generate();
    const std::vector<LineSegment>& getWalls() const;
    cv::Mat renderToImage(int cellSize, int wallThickness) const;
    void printMaze() const;
};

#endif // MAZE_GENERATOR_HPP