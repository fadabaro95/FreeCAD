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

#include "VoronoiAlgorithm.h"

#include <random>
#include <cmath>
#include <algorithm>
#include <vector>

#include <boost/polygon/voronoi.hpp>
#include <boost/geometry.hpp>

#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <Precision.hxx>

using namespace PartDesign;

std::vector<gp_Pnt2d> VoronoiAlgorithm::generateSeeds(const VoronoiParams& params)
{
    std::vector<gp_Pnt2d> points;
    int n = std::max(3, params.numPoints);
    points.reserve(n + 32);

    std::mt19937 rng(params.randomSeed);
    std::uniform_real_distribution<double> jitterDist(-0.35, 0.35);

    if (params.boundaryPolygon.size() >= 3) {
        double minX = std::numeric_limits<double>::max();
        double maxX = -std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double maxY = -std::numeric_limits<double>::max();
        for (const auto& pt : params.boundaryPolygon) {
            minX = std::min(minX, pt.X());
            maxX = std::max(maxX, pt.X());
            minY = std::min(minY, pt.Y());
            maxY = std::max(maxY, pt.Y());
        }

        double w = maxX - minX;
        double h = maxY - minY;
        if (w < 1e-4 || h < 1e-4) return points;

        double signedArea = 0.0;
        size_t numV = params.boundaryPolygon.size();
        for (size_t i = 0; i < numV; ++i) {
            size_t next = (i + 1) % numV;
            signedArea += (params.boundaryPolygon[i].X() * params.boundaryPolygon[next].Y()
                         - params.boundaryPolygon[next].X() * params.boundaryPolygon[i].Y());
        }
        double polyArea = std::abs(signedArea) * 0.5;
        double boxArea = w * h;
        double fillRatio = std::clamp(polyArea / std::max(1e-4, boxArea), 0.05, 1.0);
        int totalBoxPoints = static_cast<int>(std::round(n / fillRatio));

        double aspect = (h > 1e-4) ? (w / h) : 1.0;
        int cols = static_cast<int>(std::round(std::sqrt(totalBoxPoints * aspect)));
        if (cols < 1) cols = 1;
        int rows = static_cast<int>(std::round(static_cast<double>(totalBoxPoints) / cols));
        if (rows < 1) rows = 1;

        double dx = w / cols;
        double dy = h / rows;

        auto pointInPoly = [](const gp_Pnt2d& p, const std::vector<gp_Pnt2d>& poly) {
            int count = static_cast<int>(poly.size());
            bool inside = false;
            for (int i = 0, j = count - 1; i < count; j = i++) {
                if (((poly[i].Y() > p.Y()) != (poly[j].Y() > p.Y())) &&
                    (p.X() < (poly[j].X() - poly[i].X()) * (p.Y() - poly[i].Y()) / (poly[j].Y() - poly[i].Y()) + poly[i].X())) {
                    inside = !inside;
                }
            }
            return inside;
        };

        for (int i = 0; i < cols; ++i) {
            for (int j = 0; j < rows; ++j) {
                double jx = jitterDist(rng) * dx;
                double jy = jitterDist(rng) * dy;
                double x = minX + (i + 0.5) * dx + jx;
                double y = minY + (j + 0.5) * dy + jy;
                x = std::clamp(x, minX + 0.02 * dx, maxX - 0.02 * dx);
                y = std::clamp(y, minY + 0.02 * dy, maxY - 0.02 * dy);
                gp_Pnt2d candidate(x, y);
                if (pointInPoly(candidate, params.boundaryPolygon)) {
                    points.push_back(candidate);
                }
            }
        }

        std::uniform_real_distribution<double> randX(minX, maxX);
        std::uniform_real_distribution<double> randY(minY, maxY);
        int attempts = 0;
        while (points.size() < static_cast<size_t>(std::min(3, n)) && attempts < 1000) {
            gp_Pnt2d cand(randX(rng), randY(rng));
            if (pointInPoly(cand, params.boundaryPolygon)) {
                points.push_back(cand);
            }
            attempts++;
        }
        return points;
    }

    double aspect = (params.height > 1e-4) ? (params.width / params.height) : 1.0;
    int cols = static_cast<int>(std::round(std::sqrt(n * aspect)));
    if (cols < 1) cols = 1;
    int rows = static_cast<int>(std::round(static_cast<double>(n) / cols));
    if (rows < 1) rows = 1;

    double dx = params.width / cols;
    double dy = params.height / rows;

    // 1. Semillas internas estratificadas
    for (int i = 0; i < cols; ++i) {
        for (int j = 0; j < rows; ++j) {
            double jx = jitterDist(rng) * dx;
            double jy = jitterDist(rng) * dy;
            double x = (i + 0.5) * dx + jx;
            double y = (j + 0.5) * dy + jy;
            x = std::clamp(x, 0.05 * dx, params.width - 0.05 * dx);
            y = std::clamp(y, 0.05 * dy, params.height - 0.05 * dy);
            points.emplace_back(x, y);
        }
    }

    return points;
}

std::vector<gp_Pnt2d> VoronoiAlgorithm::clipPolygonToRect(
    const std::vector<gp_Pnt2d>& polygon,
    double xMin, double yMin, double xMax, double yMax)
{
    if (polygon.size() < 3) return {};

    auto clipEdge = [](const std::vector<gp_Pnt2d>& poly, int edgeIdx, double x0, double y0, double x1, double y1) {
        std::vector<gp_Pnt2d> output;
        if (poly.empty()) return output;

        auto isInside = [edgeIdx, x0, y0, x1, y1](const gp_Pnt2d& p) {
            switch (edgeIdx) {
                case 0: return p.X() >= x0; // Izquierda
                case 1: return p.X() <= x1; // Derecha
                case 2: return p.Y() >= y0; // Abajo
                case 3: return p.Y() <= y1; // Arriba
                default: return true;
            }
        };

        auto intersection = [edgeIdx, x0, y0, x1, y1](const gp_Pnt2d& s, const gp_Pnt2d& e) {
            double dx = e.X() - s.X();
            double dy = e.Y() - s.Y();
            if (std::abs(dx) < 1e-12 && std::abs(dy) < 1e-12) return s;

            switch (edgeIdx) {
                case 0: { // x = x0
                    double t = (x0 - s.X()) / dx;
                    return gp_Pnt2d(x0, s.Y() + t * dy);
                }
                case 1: { // x = x1
                    double t = (x1 - s.X()) / dx;
                    return gp_Pnt2d(x1, s.Y() + t * dy);
                }
                case 2: { // y = y0
                    double t = (y0 - s.Y()) / dy;
                    return gp_Pnt2d(s.X() + t * dx, y0);
                }
                case 3: { // y = y1
                    double t = (y1 - s.Y()) / dy;
                    return gp_Pnt2d(s.X() + t * dx, y1);
                }
                default: return s;
            }
        };

        gp_Pnt2d prev = poly.back();
        for (const auto& curr : poly) {
            if (isInside(curr)) {
                if (!isInside(prev)) {
                    output.push_back(intersection(prev, curr));
                }
                output.push_back(curr);
            }
            else if (isInside(prev)) {
                output.push_back(intersection(prev, curr));
            }
            prev = curr;
        }
        return output;
    };

    std::vector<gp_Pnt2d> current = polygon;
    for (int e = 0; e < 4; ++e) {
        current = clipEdge(current, e, xMin, yMin, xMax, yMax);
        if (current.size() < 3) return {};
    }
    return current;
}

std::vector<std::vector<gp_Pnt2d>> VoronoiAlgorithm::clipPolygonToBoundary(
    const std::vector<gp_Pnt2d>& subjectPoly,
    const std::vector<gp_Pnt2d>& boundaryPoly)
{
    if (subjectPoly.size() < 3 || boundaryPoly.size() < 3) return {};

    namespace bg = boost::geometry;
    using BoostPoint = bg::model::d2::point_xy<double>;
    using BoostPolygon = bg::model::polygon<BoostPoint>;
    using BoostMultiPolygon = bg::model::multi_polygon<BoostPolygon>;

    auto toBoost = [](const std::vector<gp_Pnt2d>& pts) {
        BoostPolygon poly;
        for (const auto& p : pts) {
            bg::append(poly.outer(), BoostPoint(p.X(), p.Y()));
        }
        if (!pts.empty()) {
            bg::append(poly.outer(), BoostPoint(pts.front().X(), pts.front().Y()));
        }
        bg::correct(poly);
        return poly;
    };

    BoostPolygon subj = toBoost(subjectPoly);
    BoostPolygon bound = toBoost(boundaryPoly);
    BoostMultiPolygon output;

    try {
        bg::intersection(subj, bound, output);
    }
    catch (...) {
        return {};
    }

    std::vector<std::vector<gp_Pnt2d>> result;
    for (const auto& poly : output) {
        if (bg::area(poly) < 1e-4) continue;
        const auto& ring = poly.outer();
        size_t count = ring.size();
        if (count < 4) continue;
        if (count > 1 && bg::distance(ring.front(), ring.back()) < 1e-7) {
            count--;
        }
        std::vector<gp_Pnt2d> pts;
        pts.reserve(count);
        for (size_t i = 0; i < count; ++i) {
            pts.emplace_back(ring[i].x(), ring[i].y());
        }
        if (pts.size() >= 3) {
            result.push_back(pts);
        }
    }
    return result;
}

std::vector<gp_Pnt2d> VoronoiAlgorithm::shrinkPolygon(
    const std::vector<gp_Pnt2d>& polygon,
    double offsetDistance)
{
    size_t n = polygon.size();
    if (n < 3 || offsetDistance <= 0.0) return polygon;

    // 1. Verificar orientación y calcular área con fórmula de Gauss
    double signedArea = 0.0;
    for (size_t i = 0; i < n; ++i) {
        size_t next = (i + 1) % n;
        signedArea += (polygon[i].X() * polygon[next].Y() - polygon[next].X() * polygon[i].Y());
    }

    std::vector<gp_Pnt2d> poly = polygon;
    if (signedArea < 0.0) {
        std::reverse(poly.begin(), poly.end());
        signedArea = -signedArea;
    }
    if (signedArea < 1e-4) return {};

    // 2. Líneas desplazadas hacia el interior
    struct Line2D {
        gp_Pnt2d point;
        gp_Pnt2d dir;
    };
    std::vector<Line2D> lines;
    lines.reserve(n);

    for (size_t i = 0; i < n; ++i) {
        size_t next = (i + 1) % n;
        double dx = poly[next].X() - poly[i].X();
        double dy = poly[next].Y() - poly[i].Y();
        double len = std::hypot(dx, dy);
        if (len < 1e-6) continue;

        double ux = dx / len;
        double uy = dy / len;
        // Normal hacia la izquierda para polígono CCW
        double nx = -uy;
        double ny = ux;

        gp_Pnt2d offsetPnt(poly[i].X() + nx * offsetDistance,
                           poly[i].Y() + ny * offsetDistance);
        lines.push_back({offsetPnt, gp_Pnt2d(ux, uy)});
    }

    if (lines.size() < 3) return {};

    // 3. Intersección de líneas adyacentes
    std::vector<gp_Pnt2d> shrunk;
    shrunk.reserve(lines.size());
    size_t m = lines.size();

    for (size_t i = 0; i < m; ++i) {
        size_t prev = (i + m - 1) % m;
        const auto& L1 = lines[prev];
        const auto& L2 = lines[i];

        double det = L1.dir.X() * L2.dir.Y() - L1.dir.Y() * L2.dir.X();
        if (std::abs(det) < 1e-8) {
            shrunk.push_back(L2.point);
            continue;
        }

        double dx = L2.point.X() - L1.point.X();
        double dy = L2.point.Y() - L1.point.Y();
        double t = (dx * L2.dir.Y() - dy * L2.dir.X()) / det;

        shrunk.emplace_back(L1.point.X() + t * L1.dir.X(),
                            L1.point.Y() + t * L1.dir.Y());
    }

    // 4. Validar área del polígono contraído
    double newArea = 0.0;
    for (size_t i = 0; i < shrunk.size(); ++i) {
        size_t next = (i + 1) % shrunk.size();
        newArea += (shrunk[i].X() * shrunk[next].Y() - shrunk[next].X() * shrunk[i].Y());
    }

    if (newArea <= 1e-3) return {}; // Celda colapsada

    return shrunk;
}

TopoDS_Wire VoronoiAlgorithm::buildWireFromPoints(
    const std::vector<gp_Pnt2d>& points,
    double /*filletRadius*/)
{
    if (points.size() < 3) return TopoDS_Wire();

    BRepBuilderAPI_MakeWire wireMaker;
    for (size_t i = 0; i < points.size(); ++i) {
        size_t next = (i + 1) % points.size();
        gp_Pnt p1(points[i].X(), points[i].Y(), 0.0);
        gp_Pnt p2(points[next].X(), points[next].Y(), 0.0);
        if (p1.Distance(p2) > 1e-4) {
            wireMaker.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
        }
    }

    if (!wireMaker.IsDone()) return TopoDS_Wire();
    return wireMaker.Wire();
}

std::vector<std::vector<gp_Pnt2d>> VoronoiAlgorithm::generateCellPolygons(const VoronoiParams& params)
{
    using namespace boost::polygon;

    std::vector<std::vector<gp_Pnt2d>> cellPolygons;

    std::vector<gp_Pnt2d> internalSeeds = generateSeeds(params);
    if (internalSeeds.size() < 3) return cellPolygons;

    double minX = 0.0, maxX = params.width;
    double minY = 0.0, maxY = params.height;
    bool hasCustomBoundary = (params.boundaryPolygon.size() >= 3);

    if (hasCustomBoundary) {
        minX = std::numeric_limits<double>::max();
        maxX = -std::numeric_limits<double>::max();
        minY = std::numeric_limits<double>::max();
        maxY = -std::numeric_limits<double>::max();
        for (const auto& pt : params.boundaryPolygon) {
            minX = std::min(minX, pt.X());
            maxX = std::max(maxX, pt.X());
            minY = std::min(minY, pt.Y());
            maxY = std::max(maxY, pt.Y());
        }
    }

    double spanX = maxX - minX;
    double spanY = maxY - minY;
    double margin = std::max(spanX, spanY) * 0.25;

    // Añadir anillo exterior de semillas de guarda para acotar todas las celdas internas
    std::vector<gp_Pnt2d> allSeeds = internalSeeds;
    int guardCount = 8;
    for (int g = 0; g <= guardCount; ++g) {
        double t = static_cast<double>(g) / guardCount;
        allSeeds.emplace_back(minX - margin, minY + t * spanY);
        allSeeds.emplace_back(maxX + margin, minY + t * spanY);
        allSeeds.emplace_back(minX + t * spanX, minY - margin);
        allSeeds.emplace_back(minX + t * spanX, maxY + margin);
    }

    std::vector<point_data<int32_t>> boostPoints;
    boostPoints.reserve(allSeeds.size());
    for (const auto& pt : allSeeds) {
        boostPoints.emplace_back(
            static_cast<int32_t>(std::round(pt.X() * CoordinateScale)),
            static_cast<int32_t>(std::round(pt.Y() * CoordinateScale))
        );
    }

    voronoi_diagram<double> vd;
    construct_voronoi(boostPoints.begin(), boostPoints.end(), &vd);

    double halfWall = std::max(0.01, params.wallThickness / 2.0);
    size_t internalCount = internalSeeds.size();

    // Procesar cada celda correspondiente a las semillas internas
    for (const auto& cell : vd.cells()) {
        if (cell.source_index() >= internalCount) continue; // Descartar celdas de guarda

        const auto* edge = cell.incident_edge();
        if (!edge) continue;

        std::vector<gp_Pnt2d> cellPoly;
        const auto* curr = edge;
        bool valid = true;

        do {
            if (curr->is_infinite() || !curr->vertex0() || !curr->vertex1()) {
                valid = false;
                break;
            }
            double x = curr->vertex0()->x() / CoordinateScale;
            double y = curr->vertex0()->y() / CoordinateScale;
            cellPoly.emplace_back(x, y);
            curr = curr->next();
        } while (curr != edge);

        if (!valid || cellPoly.size() < 3) continue;

        if (hasCustomBoundary) {
            // Contraer primero la celda convexa de Voronoi
            std::vector<gp_Pnt2d> shrunk = shrinkPolygon(cellPoly, halfWall);
            if (shrunk.size() < 3) continue;

            // Recortar la celda contraída contra el polígono límite
            std::vector<std::vector<gp_Pnt2d>> pieces = clipPolygonToBoundary(shrunk, params.boundaryPolygon);
            for (const auto& piece : pieces) {
                if (piece.size() >= 3) {
                    cellPolygons.push_back(piece);
                }
            }
        }
        else {
            // Recortar al rectángulo límite
            std::vector<gp_Pnt2d> clipped = clipPolygonToRect(cellPoly, 0.0, 0.0, params.width, params.height);
            if (clipped.size() < 3) continue;

            // Contraer hacia adentro para dejar el espesor de pared (ribs)
            std::vector<gp_Pnt2d> shrunk = shrinkPolygon(clipped, halfWall);
            if (shrunk.size() < 3) continue;

            cellPolygons.push_back(shrunk);
        }
    }

    return cellPolygons;
}

std::vector<TopoDS_Wire> VoronoiAlgorithm::generateCellWires(const VoronoiParams& params)
{
    std::vector<TopoDS_Wire> cellHoleWires;
    std::vector<std::vector<gp_Pnt2d>> cells = generateCellPolygons(params);
    cellHoleWires.reserve(cells.size());

    for (const auto& cellPoints : cells) {
        TopoDS_Wire cellWire = buildWireFromPoints(cellPoints, params.filletRadius);
        if (!cellWire.IsNull()) {
            cellHoleWires.push_back(cellWire);
        }
    }

    return cellHoleWires;
}

TopoDS_Shape VoronoiAlgorithm::generateVoronoi(const VoronoiParams& params)
{
    std::vector<TopoDS_Wire> cellHoleWires = generateCellWires(params);
    if (cellHoleWires.empty()) return TopoDS_Shape();

    TopoDS_Compound wiresCompound;
    BRep_Builder compoundBuilder;
    compoundBuilder.MakeCompound(wiresCompound);

    for (const auto& wire : cellHoleWires) {
        compoundBuilder.Add(wiresCompound, wire);
    }

    return wiresCompound;
}
