#pragma once
#include "mazeGeneration.hpp"
#include <algorithm>
#include <ctime>
#include <iostream>

// Point implementation
Point::Point(float x, float y) : x(x), y(y) {}

// LineSegment implementation
LineSegment::LineSegment(Point s, Point e) : start(s), end(e) {}

// Static member initialization
const int MazeGenerator::dx[4] = { 0, 1, 0, -1 };
const int MazeGenerator::dy[4] = { -1, 0, 1, 0 };

// MazeGenerator implementation
MazeGenerator::MazeGenerator(int w, int h, unsigned seed) : width(w), height(h) {
    if (seed == 0) {
        seed = static_cast<unsigned>(time(nullptr));
    }
    rng.seed(seed);
    visited.resize(height, std::vector<bool>(width, false));
}

bool MazeGenerator::isValid(int x, int y) {
    return x >= 0 && x < width && y >= 0 && y < height;
}

void MazeGenerator::generate() {
    walls.clear();
    visited.assign(height, std::vector<bool>(width, false));

    std::vector<std::vector<bool>> rightWall(height, std::vector<bool>(width, true));
    std::vector<std::vector<bool>> bottomWall(height, std::vector<bool>(width, true));

    carvePassagesWithWalls(0, 0, rightWall, bottomWall);

    // Convert grid to line segments
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

void MazeGenerator::carvePassagesWithWalls(int x, int y,
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

const std::vector<LineSegment>& MazeGenerator::getWalls() const {
    return walls;
}

cv::Mat MazeGenerator::renderToImage(int cellSize, int wallThickness) const {
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

void MazeGenerator::printMaze() const {
    // Print top border
    for (int x = 0; x < width; x++) {
        std::cout << "+---";
    }
    std::cout << "+\n";

    // Print maze
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

int MazeGenerator::getWidth()
{
    return width;
}

int MazeGenerator::getHeight()
{
    return height;
}