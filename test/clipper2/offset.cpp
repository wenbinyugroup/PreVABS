#include <iostream>
#include <vector>
#include <cmath>
// Clipper2 headers:
#include "clipper2/clipper.offset.h"   // for ClipperOffset
#include "clipper2/clipper.h"          // for PathD, PathsD, etc.

// A small struct for convenience
struct PointD {
    double x;
    double y;
};

//
// Helper: compute orientation of a polyline
// Returns +1 if polyline is oriented CCW (common for a->b->c, etc.)
// Returns -1 if orientation is CW
//
double Orientation(const std::vector<PointD>& path)
{
    // Simple area-based orientation test
    // (Shoelace formula for polygon area but ignoring closing)
    double area = 0.0;
    for (size_t i = 0; i + 1 < path.size(); i++) {
        area += (path[i].x * path[i+1].y) - (path[i+1].x * path[i].y);
    }
    // ccw => area > 0
    return (area >= 0.0) ? +1.0 : -1.0;
}

//
// Helper: convert from std::vector<PointD> to Clipper2::PathD
//
Clipper2Lib::PathD ToClipperPath(const std::vector<PointD>& pts)
{
    Clipper2Lib::PathD result;
    result.reserve(pts.size());
    for (auto &p : pts) {
        result.push_back(Clipper2Lib::PointD(p.x, p.y));
    }
    return result;
}

//
// Helper: convert Clipper2::PathD back to std::vector<PointD>
//
std::vector<PointD> FromClipperPath(const Clipper2Lib::PathD& path)
{
    std::vector<PointD> result;
    result.reserve(path.size());
    for (auto &p : path) {
        result.push_back({ p.x, p.y });
    }
    return result;
}

//
// Extend or trim the endpoint of a line segment [pt0->pt1] so that it lies on the 
// direction line from pt0 along 'directionVector'.
//
// lengthMode > 0: extend or trim to exactly lengthMode distance from pt0
// lengthMode <= 0: purely directional, but typically you'd find an intersection 
//                  or have some logic to find how far to extend/trim
//
PointD ExtendOrTrimEnd(const PointD &pt0,
                       const PointD &pt1,
                       const PointD &directionVector,
                       double lengthMode = 0.0)
{
    // directionVector = e.g. 'ac' if user wants to define the direction from 'a'
    // that the offset line should approach or meet.
    // This is just a simplistic example:
    
    // Normalize direction vector:
    double dx = directionVector.x;
    double dy = directionVector.y;
    double mag = std::sqrt(dx*dx + dy*dy);
    if (mag < 1e-12) {
        // If direction vector is invalid or zero, just return the original pt1
        return pt1;
    }
    dx /= mag;
    dy /= mag;

    // If lengthMode > 0, place the new endpoint lengthMode away from pt0
    // along the direction vector.
    if (lengthMode > 0) {
        PointD result;
        result.x = pt0.x + dx * lengthMode;
        result.y = pt0.y + dy * lengthMode;
        return result;
    } 
    else {
        // Or do something more advanced, like line intersection, etc.
        // For now, just return the original point.
        return pt1;
    }
}

//
// MAIN offsetting function
//
// Params:
//   baseLine: the input open line, e.g., [a, ..., A]
//   offsetDist: the offset distance > 0
//   offsetLeft: if true, offset "to the left" (relative to line orientation); 
//               if false, offset to the right
//   endVectorFirst, endVectorLast: optional custom direction vectors for the 
//               first and last endpoints. If they have length > 0, we can 
//               do a custom extension/trim.
//
// Returns: The offset line as a list of points
//
std::vector<PointD> OffsetOpenLine(const std::vector<PointD> &baseLine,
                                   double offsetDist,
                                   bool offsetLeft,
                                   const PointD &endVectorFirst = {0,0},
                                   const PointD &endVectorLast  = {0,0})
{
    if (baseLine.size() < 2) {
        // degenerate input
        return baseLine;
    }
    
    // 1. Check orientation of baseLine
    double orient = Orientation(baseLine);
    // If user wants an offset to the "left" but the orientation is already CCW, 
    // we keep offsetDist positive. If user wants "right" but orientation is CCW, 
    // we can use negative offsetDist => effectively offset the other side.
    // Or you can just forcibly re-orient the baseLine to be in a known direction, 
    // then apply positive or negative offsetDist accordingly.

    double signedOffsetDist = offsetDist;
    bool wantCCW = offsetLeft;  // if we want offset left, prefer CCW path
    bool isCCW = (orient > 0.0);

    // If the orientation is not what we want, we can reverse the path
    // so that applying a positive offset will go the correct side.
    std::vector<PointD> orientedLine = baseLine;
    if (isCCW != wantCCW) {
        std::reverse(orientedLine.begin(), orientedLine.end());
    }

    // Convert to Clipper2 path
    Clipper2Lib::PathD pathD = ToClipperPath(orientedLine);
    
    // 2. Create a ClipperOffset object
    Clipper2Lib::ClipperOffset offsetter;

    // 3. Add path with the desired join type and end type
    //    Common choices for open lines:
    //    JoinType::Miter (or Round/Square) and EndType::Butt (or Round/Square)
    using namespace Clipper2Lib;
    offsetter.AddPath(
        pathD,
        JoinType::Miter,     // how corners are joined
        EndType::Butt        // how open ends are shaped
    );

    // 4. Perform the offset
    PathsD solution;
    offsetter.Execute(solution, offsetDist); 

    // For an open path, solution should typically have 1 path if offsetDist 
    // is large enough to offset in a single direction. 
    // But check in case of edge cases or self-intersections.
    if (solution.empty()) {
        // No offset result (possibly offsetDist too large for short segment)
        return {};
    }

    // We'll assume the first path in solution is our offset line:
    PathD offsetPath = solution.front();
    auto offsetLineOut = FromClipperPath(offsetPath);

    // 5. If we reversed the line at the start, we may want to re-reverse 
    //    the offset result so that it lines up with the original direction
    if (isCCW != wantCCW) {
        std::reverse(offsetLineOut.begin(), offsetLineOut.end());
    }

    // 6. If the user provided custom end vectors for the first or last endpoints, 
    //    do a step to project/trim the offset line. 
    //    Here is just a minimal example showing how you might call the helper:

    // Original endpoints (a, A)
    PointD a = baseLine.front();
    PointD A = baseLine.back();

    // Now offset endpoints (b, B)
    PointD &b = offsetLineOut.front();
    PointD &B = offsetLineOut.back();

    // For example, if endVectorFirst != (0,0), let's say we want to
    // “trim or extend” at the first endpoint with some logic:
    if (std::fabs(endVectorFirst.x) > 1e-12 || std::fabs(endVectorFirst.y) > 1e-12) {
        // This is just a demonstration. You might do intersection 
        // or a more sophisticated approach:
        b = ExtendOrTrimEnd(a, b, endVectorFirst, 0.0);
    }

    if (std::fabs(endVectorLast.x) > 1e-12 || std::fabs(endVectorLast.y) > 1e-12) {
        B = ExtendOrTrimEnd(A, B, endVectorLast, 0.0);
    }

    // 7. If the user wants lines (a->b) and (A->B) to “close the ends” 
    //    in a simple polygon, you can add those segments manually in the 
    //    final shape. But for the “offset line” alone, we can just return 
    //    the path as is.

    return offsetLineOut;
}


//----------------------------------------------------
// Example usage
//----------------------------------------------------
int main()
{
    // Suppose we have an open line with points a -> ... -> A
    std::vector<PointD> baseLine = { {0,0}, {10,0}, {15,5} };
    
    double offsetDist = 2.0;
    bool offsetLeft = true;  // offset to the "left" side
    // No special end vectors for this example
    PointD endVectorFirst = {0,0};
    PointD endVectorLast  = {0,0};
    
    auto resultLine = OffsetOpenLine(baseLine, offsetDist, offsetLeft,
                                     endVectorFirst, endVectorLast);

    // Print the offset line
    std::cout << "Offset line:\n";
    for (auto &p : resultLine) {
        std::cout << "(" << p.x << ", " << p.y << ")\n";
    }

    return 0;
}
