// SPDX-License-Identifier: LGPL-2.1-or-later
/***************************************************************************
 *   Copyright (c) 2026 FreeCAD Developers                                 *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 ***************************************************************************/

#pragma once

#include <vector>
#include <gp_Pnt2d.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <Mod/PartDesign/PartDesignGlobal.h>

namespace PartDesign
{

/**
 * @brief Parameters for generative Voronoi structure calculation.
 */
struct VoronoiParams
{
    int numPoints {60};             ///< Approximate number of internal seed points
    double wallThickness {2.0};     ///< Rib / wall thickness between cells in mm
    double filletRadius {1.0};      ///< Corner fillet radius in mm
    double depth {5.0};             ///< Extrusion depth / height in mm
    int randomSeed {1234};          ///< RNG seed for deterministic reproduction
    double width {100.0};           ///< Bounding rectangle width in mm
    double height {100.0};          ///< Bounding rectangle height in mm
    bool isAdditive {true};         ///< True = solid ribs (Additive); False = hole prisms (Subtractive)
};

/**
 * @brief Algorithmic core for generating 2D/2.5D Voronoi cell geometry and solids.
 *
 * Utilizes Boost.Polygon Voronoi (Fortune's sweep-line) with exact integer scaling,
 * Sutherland-Hodgman polygon clipping, and OpenCASCADE BRep modeling.
 */
class PartDesignExport VoronoiAlgorithm
{
public:
    VoronoiAlgorithm() = default;

    /**
     * @brief Generates the 3D TopoDS_Shape representing the Voronoi structure.
     * @param params Generation parameters.
     * @return TopoDS_Shape (Compound of wires representing Voronoi cells).
     */
    static TopoDS_Shape generateVoronoi(const VoronoiParams& params);

    /**
     * @brief Generates the list of contracted 2D polygon vertices for all internal Voronoi cells.
     */
    static std::vector<std::vector<gp_Pnt2d>> generateCellPolygons(const VoronoiParams& params);

    /**
     * @brief Generates a vector of closed TopoDS_Wire, one for each internal Voronoi cell.
     */
    static std::vector<TopoDS_Wire> generateCellWires(const VoronoiParams& params);

    /**
     * @brief Generates distributed 2D seed points using stratified jittered grid.
     */
    static std::vector<gp_Pnt2d> generateSeeds(const VoronoiParams& params);

    /**
     * @brief Sutherland-Hodgman 2D convex polygon clipping against a rectangle.
     */
    static std::vector<gp_Pnt2d> clipPolygonToRect(
        const std::vector<gp_Pnt2d>& polygon,
        double xMin, double yMin, double xMax, double yMax
    );

    /**
     * @brief Shrinks a 2D convex polygon inward by a specified distance (half wall thickness).
     */
    static std::vector<gp_Pnt2d> shrinkPolygon(
        const std::vector<gp_Pnt2d>& polygon,
        double offsetDistance
    );

    /**
     * @brief Builds a closed TopoDS_Wire from a list of 2D points on the XY plane.
     */
    static TopoDS_Wire buildWireFromPoints(
        const std::vector<gp_Pnt2d>& points,
        double filletRadius = 0.0
    );

private:
    static constexpr double CoordinateScale = 10000.0; // Scale factor for Boost.Polygon integer grid
};

} // namespace PartDesign
