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

#include "VoronoiSketch.h"
#include "VoronoiAlgorithm.h"

#include <Standard_Failure.hxx>
#include <Precision.hxx>
#include <Mod/Part/App/Geometry.h>
#include <Mod/Part/App/Part2DObject.h>

#include <BRepTools_WireExplorer.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_UniformDeflection.hxx>
#include <BRepOffsetAPI_MakeOffset.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <BRep_Tool.hxx>

using namespace PartDesign;

static const App::PropertyQuantityConstraint::Constraints quantityRangePositive = {
    2 * Precision::Confusion(), std::numeric_limits<float>::max(), 0.1
};

PROPERTY_SOURCE(PartDesign::VoronoiSketch, Sketcher::SketchObject)

VoronoiSketch::VoronoiSketch()
{
    ADD_PROPERTY_TYPE(NumberOfSeeds, (50), "Voronoi", App::Prop_None, "Number of cell seeds");
    ADD_PROPERTY_TYPE(WallThickness, (2.0f), "Voronoi", App::Prop_None, "Thickness of ribs between cells");
    ADD_PROPERTY_TYPE(FilletRadius, (0.5f), "Voronoi", App::Prop_None, "Corner fillet radius inside cells");
    ADD_PROPERTY_TYPE(Width, (100.0f), "Voronoi", App::Prop_None, "Width of the bounding domain");
    ADD_PROPERTY_TYPE(Height, (100.0f), "Voronoi", App::Prop_None, "Height of the bounding domain");
    ADD_PROPERTY_TYPE(SeedRandom, (1234), "Voronoi", App::Prop_None, "Random seed for reproduction");
    ADD_PROPERTY_TYPE(CloseContour, (true), "Voronoi", App::Prop_None, "Close outer boundary contour of the sketch");
    ADD_PROPERTY_TYPE(BoundarySketch, (nullptr), "Voronoi", App::Prop_None, "Optional sketch or Part feature with a closed contour to bound the Voronoi pattern");

    WallThickness.setConstraints(&quantityRangePositive);
    Width.setConstraints(&quantityRangePositive);
    Height.setConstraints(&quantityRangePositive);
}

void VoronoiSketch::onChanged(const App::Property* prop)
{
    if (prop == &BoundarySketch) {
        auto* baseObj = freecad_cast<Part::Part2DObject*>(BoundarySketch.getValue());
        if (baseObj && AttachmentSupport.getValues().empty()) {
            AttachmentSupport.setValues(baseObj->AttachmentSupport.getValues(),
                                        baseObj->AttachmentSupport.getSubValues());
            MapMode.setValue(baseObj->MapMode.getValue());
            MapReversed.setValue(baseObj->MapReversed.getValue());
            AttachmentOffset.setValue(baseObj->AttachmentOffset.getValue());
            Placement.setValue(baseObj->Placement.getValue());
        }
    }
    SketchObject::onChanged(prop);
}

short VoronoiSketch::mustExecute() const
{
    if (NumberOfSeeds.isTouched() || WallThickness.isTouched() || FilletRadius.isTouched()
        || Width.isTouched() || Height.isTouched() || SeedRandom.isTouched()
        || CloseContour.isTouched() || BoundarySketch.isTouched()) {
        return 1;
    }
    return SketchObject::mustExecute();
}

App::DocumentObjectExecReturn* VoronoiSketch::execute()
{
    double w = Width.getValue();
    double h = Height.getValue();
    double t = WallThickness.getValue();
    double r = FilletRadius.getValue();
    int seeds = static_cast<int>(NumberOfSeeds.getValue());
    int seedRng = static_cast<int>(SeedRandom.getValue());

    if (t < Precision::Confusion()) {
        return new App::DocumentObjectExecReturn(
            QT_TRANSLATE_NOOP("Exception", "Wall thickness must be greater than zero")
        );
    }

    try {
        VoronoiParams params;
        params.numPoints = std::max(3, seeds);
        params.wallThickness = t;
        params.filletRadius = r;
        params.randomSeed = seedRng;
        params.closeContour = CloseContour.getValue();

        auto* boundObj = freecad_cast<Part::Feature*>(BoundarySketch.getValue());
        TopoDS_Wire boundaryWire;
        bool hasBoundarySketch = false;

        if (boundObj) {
            // Relocate boundary shape to VoronoiSketch's local coordinate system
            Base::Placement pThis = App::GeoFeature::getGlobalPlacement(this);
            Base::Placement pBound = App::GeoFeature::getGlobalPlacement(boundObj);
            Base::Placement relPl = pThis.inverse() * pBound;
            gp_Trsf trsf = Part::TopoShape::convert(relPl.toMatrix());
            TopoDS_Shape localShape = Part::TopoShape::moved(boundObj->Shape.getShape().getShape(), trsf);

            std::vector<TopoDS_Wire> wires;
            TopExp_Explorer exp(localShape, TopAbs_WIRE);
            for (; exp.More(); exp.Next()) {
                wires.push_back(TopoDS::Wire(exp.Current()));
            }

            if (wires.empty()) {
                BRepBuilderAPI_MakeWire wm;
                for (TopExp_Explorer eExp(localShape, TopAbs_EDGE); eExp.More(); eExp.Next()) {
                    wm.Add(TopoDS::Edge(eExp.Current()));
                }
                if (wm.IsDone()) {
                    wires.push_back(wm.Wire());
                }
            }

            auto isClosed = [](const TopoDS_Wire& wire) {
                if (wire.Closed()) return true;
                TopoDS_Vertex v1, v2;
                TopExp::Vertices(wire, v1, v2);
                if (!v1.IsNull() && !v2.IsNull()) {
                    return BRep_Tool::Pnt(v1).Distance(BRep_Tool::Pnt(v2)) < 1e-3;
                }
                return false;
            };

            std::vector<TopoDS_Wire> closedWires;
            for (const auto& wire : wires) {
                if (isClosed(wire)) {
                    closedWires.push_back(wire);
                }
            }

            if (closedWires.empty()) {
                return new App::DocumentObjectExecReturn(
                    QT_TRANSLATE_NOOP("Exception", "Boundary sketch does not contain a closed contour")
                );
            }

            // Select closed wire with largest bounding box area
            boundaryWire = closedWires[0];
            double maxArea = -1.0;
            for (const auto& cw : closedWires) {
                Bnd_Box b;
                BRepBndLib::Add(cw, b);
                if (!b.IsVoid()) {
                    double x0, y0, z0, x1, y1, z1;
                    b.Get(x0, y0, z0, x1, y1, z1);
                    double a = (x1 - x0) * (y1 - y0);
                    if (a > maxArea) {
                        maxArea = a;
                        boundaryWire = cw;
                    }
                }
            }

            // Discretize boundaryWire into 2D polygon
            std::vector<gp_Pnt2d> contourPoints;
            for (BRepTools_WireExplorer we(boundaryWire); we.More(); we.Next()) {
                TopoDS_Edge e = we.Current();
                BRepAdaptor_Curve adapt(e);
                double f = adapt.FirstParameter();
                double l = adapt.LastParameter();
                if (adapt.GetType() == GeomAbs_Line) {
                    gp_Pnt p = adapt.Value(f);
                    contourPoints.emplace_back(p.X(), p.Y());
                }
                else {
                    GCPnts_UniformDeflection discretizer(adapt, 0.1, f, l);
                    if (discretizer.IsDone() && discretizer.NbPoints() > 0) {
                        int nb = discretizer.NbPoints();
                        for (int i = 1; i <= nb; ++i) {
                            if (i == nb && we.More()) continue;
                            gp_Pnt p = discretizer.Value(i);
                            contourPoints.emplace_back(p.X(), p.Y());
                        }
                    }
                    else {
                        int samples = 16;
                        for (int i = 0; i < samples; ++i) {
                            double param = f + (l - f) * (static_cast<double>(i) / samples);
                            gp_Pnt p = adapt.Value(param);
                            contourPoints.emplace_back(p.X(), p.Y());
                        }
                    }
                }
            }

            // Remove adjacent duplicates
            std::vector<gp_Pnt2d> cleanContour;
            for (const auto& pt : contourPoints) {
                if (cleanContour.empty() || cleanContour.back().Distance(pt) > 1e-4) {
                    cleanContour.push_back(pt);
                }
            }
            if (!cleanContour.empty() && cleanContour.front().Distance(cleanContour.back()) < 1e-4) {
                cleanContour.pop_back();
            }

            if (cleanContour.size() >= 3) {
                // Ensure counter-clockwise vertex orientation
                double signedArea = 0.0;
                size_t numV = cleanContour.size();
                for (size_t i = 0; i < numV; ++i) {
                    size_t next = (i + 1) % numV;
                    signedArea += (cleanContour[i].X() * cleanContour[next].Y()
                                 - cleanContour[next].X() * cleanContour[i].Y());
                }
                if (signedArea < 0.0) {
                    std::reverse(cleanContour.begin(), cleanContour.end());
                }

                // Inward offset of boundary wire by halfWall to leave outer rib margin
                double halfWall = t / 2.0;
                std::vector<gp_Pnt2d> clippingContour = cleanContour;
                try {
                    BRepOffsetAPI_MakeOffset mkOffset;
                    mkOffset.Init(GeomAbs_Arc);
                    mkOffset.AddWire(boundaryWire);
                    mkOffset.Perform(-halfWall);
                    if (mkOffset.IsDone() && !mkOffset.Shape().IsNull()) {
                        TopExp_Explorer expO(mkOffset.Shape(), TopAbs_WIRE);
                        if (expO.More()) {
                            TopoDS_Wire shrunkWire = TopoDS::Wire(expO.Current());
                            std::vector<gp_Pnt2d> shrunkPts;
                            for (BRepTools_WireExplorer we(shrunkWire); we.More(); we.Next()) {
                                TopoDS_Edge e = we.Current();
                                BRepAdaptor_Curve adapt(e);
                                double f = adapt.FirstParameter();
                                double l = adapt.LastParameter();
                                if (adapt.GetType() == GeomAbs_Line) {
                                    gp_Pnt p = adapt.Value(f);
                                    shrunkPts.emplace_back(p.X(), p.Y());
                                }
                                else {
                                    GCPnts_UniformDeflection discretizer(adapt, 0.1, f, l);
                                    if (discretizer.IsDone() && discretizer.NbPoints() > 0) {
                                        for (int i = 1; i <= discretizer.NbPoints(); ++i) {
                                            gp_Pnt p = discretizer.Value(i);
                                            shrunkPts.emplace_back(p.X(), p.Y());
                                        }
                                    }
                                }
                            }
                            if (shrunkPts.size() >= 3) {
                                clippingContour = shrunkPts;
                            }
                        }
                    }
                }
                catch (...) {
                }

                params.boundaryPolygon = clippingContour;
                hasBoundarySketch = true;
            }
        }

        if (!hasBoundarySketch) {
            if (w < Precision::Confusion() || h < Precision::Confusion()) {
                return new App::DocumentObjectExecReturn(
                    QT_TRANSLATE_NOOP("Exception", "Voronoi dimensions are too small")
                );
            }
            params.width = w;
            params.height = h;
        }

        std::vector<std::vector<gp_Pnt2d>> cells = VoronoiAlgorithm::generateCellPolygons(params);

        std::vector<Part::Geometry*> newGeom;
        newGeom.reserve(cells.size() * 6 + 16);

        if (CloseContour.getValue()) {
            if (hasBoundarySketch && !boundaryWire.IsNull()) {
                // Copy exact geometric curves/lines from the boundary sketch
                for (BRepTools_WireExplorer we(boundaryWire); we.More(); we.Next()) {
                    TopoDS_Edge e = we.Current();
                    auto geom = Part::Geometry::fromShape(e);
                    if (geom) {
                        newGeom.push_back(geom.release());
                    }
                }
            }
            else {
                gp_Pnt2d corners[4] = {
                    gp_Pnt2d(0.0, 0.0),
                    gp_Pnt2d(w, 0.0),
                    gp_Pnt2d(w, h),
                    gp_Pnt2d(0.0, h)
                };
                for (int i = 0; i < 4; ++i) {
                    int next = (i + 1) % 4;
                    auto* line = new Part::GeomLineSegment();
                    line->setPoints(
                        Base::Vector3d(corners[i].X(), corners[i].Y(), 0.0),
                        Base::Vector3d(corners[next].X(), corners[next].Y(), 0.0)
                    );
                    newGeom.push_back(line);
                }
            }
        }

        for (const auto& cell : cells) {
            size_t n = cell.size();
            for (size_t i = 0; i < n; ++i) {
                size_t next = (i + 1) % n;
                gp_Pnt2d p1 = cell[i];
                gp_Pnt2d p2 = cell[next];
                if (p1.Distance(p2) > 1e-4) {
                    auto* line = new Part::GeomLineSegment();
                    line->setPoints(
                        Base::Vector3d(p1.X(), p1.Y(), 0.0),
                        Base::Vector3d(p2.X(), p2.Y(), 0.0)
                    );
                    newGeom.push_back(line);
                }
            }
        }

        // Update geometry in the sketch
        Geometry.setValues(newGeom);
        for (auto* g : newGeom) {
            delete g;
        }

        // Delegate to base SketchObject to position by support and build Shape
        return SketchObject::execute();
    }
    catch (Standard_Failure& e) {
        return new App::DocumentObjectExecReturn(e.GetMessageString());
    }
    catch (const Base::Exception& e) {
        return new App::DocumentObjectExecReturn(e.what());
    }
}

