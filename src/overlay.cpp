#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
//#include <cv_bridge/cv_bridge.h>

using namespace cv;

int main(int argc, char** argv) {
    // Load the map images
    Mat worldMap = imread(argv[1], IMREAD_COLOR);
    Mat slamMap = imread(argv[2], IMREAD_GRAYSCALE);
    

    // Determine the dimensions of the world map
    int minX = 1000;
    int minY = 1000;
    int maxX = 0;
    int maxY = 0;

    Mat worldGray = imread(argv[1], IMREAD_GRAYSCALE);
    int rows = worldGray.rows;
    int cols = worldGray.cols;

    for (int x = 0; x < rows; x++) {
        for (int y = 0; y < cols; y++) {
            // Check each black pixel
            if (worldGray.at<uchar>(x, y) == 0) {
                // Update minimum points
                if (x < minX) {
                    minX = x;
                }
                if (y < minY) {
                    minY = y;
                }

                // Update maximum points
                if (x > maxX) {
                    maxX = x;
                }
                if (y > maxY) {
                    maxY = y;
                }
            }
        }
    }

    int newWidth = maxX - minX;
    int newHeight = maxY - minY;

    Mat resizedSlam;
    cv::resize(slamMap, resizedSlam, cv::Size(newHeight, newWidth), 0, 0, cv::INTER_AREA);

    //Overlay the Images 

    Mat finalOverlay = worldMap.clone(); // Create a copy for the final image
    
    // World image midpoints
    int worldRows = finalOverlay.rows;
    int worldCols = finalOverlay.cols;

    int worldMidRow = worldRows / 2;
    int worldMidCol = worldCols / 2;

    // SLAM image midpoints
    int slamRows = resizedSlam.rows;
    int slamCols = resizedSlam.cols;

    int slamMidRow = slamRows / 2;
    int slamMidCol = slamCols / 2;

    // Calculate starting and ending points for overlay
    int startRow = worldMidRow - slamMidRow;
    int startCol = worldMidCol - slamMidCol;

    int endRow = worldMidRow + slamMidRow;
    int endCol = worldMidCol + slamMidCol;

    // Loop through pixels to blend the images
    for (int x = 0; x < worldRows; x++) {
        for (int y = 0; y < worldCols; y++) {
            if ((x >= startRow && x <= endRow) && (y >= startCol && y <= endCol)) {
                finalOverlay.at<Vec3b>(x, y)[0] = (0.25 * worldMap.at<Vec3b>(x, y)[0]) +
                (0.75 * resizedSlam.at<uchar>(x - startRow, y - startCol));
    
            } 

            if (x > 900) {
                std::cout << "Processing pixel at x: " << x << std::endl;
            }
        }
    }

    imshow("Overlayed Map", finalOverlay);
    waitKey(0);

    return 0;
}