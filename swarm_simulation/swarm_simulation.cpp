#include "mazeGeneration.hpp"
#include <iostream>

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