///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 2
// Authors: Shlok Sobti
//////////////////////////////////////

#include "CollisionChecking.h"


// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{

    for (size_t i = 0; i < (obstacles.size()); i++) {

      double xMin = obstacles[i].x;
      double yMin = obstacles[i].y;
      double xMax = obstacles[i].x + obstacles[i].width;
      double yMax = obstacles[i].y + obstacles[i].height;

      if ((x>=xMin && x<=xMax) && (y>=yMin && y<=yMax)) {
        return false;
      }
    }
    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles. If the circle lies outside of all
// obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
      for (size_t i = 0; i < (obstacles.size()); i++) {

        double xMin = obstacles[i].x;
        double yMin = obstacles[i].y;
        double xMax = obstacles[i].x + obstacles[i].width;
        double yMax = obstacles[i].y + obstacles[i].height;

        bool C1 = (x >= (xMin-radius)) && (x <= (xMax+radius));
        bool C2 = y >= yMin && y <= yMax;
        bool C3 = x >= xMin && x <= xMax;
        bool C4 = (y >= (yMin-radius)) && (y <= (yMax+radius));

        bool vertexCond1 =  radius >= sqrt(pow((x-xMin),2) + pow((y-yMin),2));
        bool vertexCond2 =  radius >= sqrt(pow((x-xMax),2) + pow((y-yMin),2));
        bool vertexCond3 =  radius >= sqrt(pow((x-xMin),2) + pow((y-yMax),2));
        bool vertexCond4 =  radius >= sqrt(pow((x-xMax),2) + pow((y-yMax),2));

        bool C5 = vertexCond1 || vertexCond2 || vertexCond3 || vertexCond4;

        if ((C1 && C2) || (C3 && C4) || C5) {
          return false;
        }
      }
      return true;
}

// Function that returns true if two line segments intersect
bool doesIntersect(double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4){
      double Denominator = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1);
      double tA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / Denominator;
      double tB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / Denominator;

      if ((y2-y1)/(x2-x1) == (y4-y3)/(x4-x3)) {
        return false; // Parallel or Co-linear
      } else if (tA < 0 || tA > 1 || tB < 0 || tB > 1) {
        return false;
      }
      return true;
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{

    // Step 1: Map the local positions of the square vertices to the global frame.
    double vertexA_xCoordinate = -(sideLength/2)*cos(theta) + (sideLength/2)*sin(theta) + x;
    double vertexA_yCoordinate = -(sideLength/2)*sin(theta) - (sideLength/2)*cos(theta) + y;

    double vertexB_xCoordinate = -(sideLength/2)*cos(theta) - (sideLength/2)*sin(theta) + x;
    double vertexB_yCoordinate = -(sideLength/2)*sin(theta) + (sideLength/2)*cos(theta) + y;

    double vertexC_xCoordinate = (sideLength/2)*cos(theta) - (sideLength/2)*sin(theta) + x;
    double vertexC_yCoordinate = (sideLength/2)*sin(theta) + (sideLength/2)*cos(theta) + y;

    double vertexD_xCoordinate = (sideLength/2)*cos(theta) + (sideLength/2)*sin(theta) + x;
    double vertexD_yCoordinate = (sideLength/2)*sin(theta) - (sideLength/2)*cos(theta) + y;

    for (size_t i = 0; i < (obstacles.size()); i++) {

      double P_x = obstacles[i].x;
      double P_y = obstacles[i].y;
      double Q_x = obstacles[i].x;
      double Q_y = obstacles[i].y + obstacles[i].height;
      double R_x = obstacles[i].x + obstacles[i].width;
      double R_y = obstacles[i].y + obstacles[i].height;
      double S_x = obstacles[i].x + obstacles[i].width;
      double S_y = obstacles[i].y;

      // Check if Robot inside obstacle
      if (vertexA_xCoordinate < S_x && vertexA_xCoordinate > P_x && vertexA_yCoordinate < Q_y && vertexA_yCoordinate > P_y) {
        return false;
      }

      // AB intersects PQ
      if (doesIntersect(vertexA_xCoordinate, vertexA_yCoordinate, vertexB_xCoordinate, vertexB_yCoordinate, P_x, P_y, Q_x, Q_y)) {
        return false;
      }
      // AB intersects QR
      if (doesIntersect(vertexA_xCoordinate, vertexA_yCoordinate, vertexB_xCoordinate, vertexB_yCoordinate, Q_x, Q_y, R_x, R_y)) {
        return false;
      }
      // AB intersects RS
      if (doesIntersect(vertexA_xCoordinate, vertexA_yCoordinate, vertexB_xCoordinate, vertexB_yCoordinate, R_x, R_y, S_x, S_y)) {
        return false;
      }
      // AB intersects SP
      if (doesIntersect(vertexA_xCoordinate, vertexA_yCoordinate, vertexB_xCoordinate, vertexB_yCoordinate, S_x, S_y, P_x, P_y)) {
        return false;
      }

      // BC intersects PQ
      if (doesIntersect(vertexB_xCoordinate, vertexB_yCoordinate, vertexC_xCoordinate, vertexC_yCoordinate, P_x, P_y, Q_x, Q_y)) {
        return false;
      }
      // BC intersects QR
      if (doesIntersect(vertexB_xCoordinate, vertexB_yCoordinate, vertexC_xCoordinate, vertexC_yCoordinate, Q_x, Q_y, R_x, R_y)) {
        return false;
      }
      // BC intersects RS
      if (doesIntersect(vertexB_xCoordinate, vertexB_yCoordinate, vertexC_xCoordinate, vertexC_yCoordinate, R_x, R_y, S_x, S_y)) {
        return false;
      }
      // BC intersects SP
      if (doesIntersect(vertexB_xCoordinate, vertexB_yCoordinate, vertexC_xCoordinate, vertexC_yCoordinate, S_x, S_y, P_x, P_y)) {
        return false;
      }

      // CD intersects PQ
      if (doesIntersect(vertexC_xCoordinate, vertexC_yCoordinate, vertexD_xCoordinate, vertexD_yCoordinate, P_x, P_y, Q_x, Q_y)) {
        return false;
      }
      // CD intersects QR
      if (doesIntersect(vertexC_xCoordinate, vertexC_yCoordinate, vertexD_xCoordinate, vertexD_yCoordinate, Q_x, Q_y, R_x, R_y)) {
        return false;
      }
      // CD intersects RS
      if (doesIntersect(vertexC_xCoordinate, vertexC_yCoordinate, vertexD_xCoordinate, vertexD_yCoordinate, R_x, R_y, S_x, S_y)) {
        return false;
      }
      // CD intersects SP
      if (doesIntersect(vertexC_xCoordinate, vertexC_yCoordinate, vertexD_xCoordinate, vertexD_yCoordinate, S_x, S_y, P_x, P_y)) {
        return false;
      }

      // DA intersects PQ
      if (doesIntersect(vertexD_xCoordinate, vertexD_yCoordinate, vertexA_xCoordinate, vertexA_yCoordinate, P_x, P_y, Q_x, Q_y)) {
        return false;
      }
      // DA intersects QR
      if (doesIntersect(vertexD_xCoordinate, vertexD_yCoordinate, vertexA_xCoordinate, vertexA_yCoordinate, Q_x, Q_y, R_x, R_y)) {
        return false;
      }
      // DA intersects RS
      if (doesIntersect(vertexD_xCoordinate, vertexD_yCoordinate, vertexA_xCoordinate, vertexA_yCoordinate, R_x, R_y, S_x, S_y)) {
        return false;
      }
      // DA intersects SP
      if (doesIntersect(vertexD_xCoordinate, vertexD_yCoordinate, vertexA_xCoordinate, vertexA_yCoordinate, S_x, S_y, P_x, P_y)) {
        return false;
      }
    }
    return true;
}

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
//void debugMode(const std::vector<Robot> & /*robots*/, const std::vector<Rectangle> & /*obstacles*/,
//               const std::vector<bool> & /*valid*/)



//}
