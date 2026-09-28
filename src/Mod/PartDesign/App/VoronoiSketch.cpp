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

    WallThickness.setConstraints(&quantityRangePositive);
    Width.setConstraints(&quantityRangePositive);
    Height.setConstraints(&quantityRangePositive);
}

void VoronoiSketch::onChanged(const App::Property* prop)
{
    SketchObject::onChanged(prop);
}

short VoronoiSketch::mustExecute() const
{
    if (NumberOfSeeds.isTouched() || WallThickness.isTouched() || FilletRadius.isTouched()
        || Width.isTouched() || Height.isTouched() || SeedRandom.isTouched()) {
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

    if (w < Precision::Confusion() || h < Precision::Confusion()) {
        return new App::DocumentObjectExecReturn(
            QT_TRANSLATE_NOOP("Exception", "Voronoi dimensions are too small")
        );
    }

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
        params.width = w;
        params.height = h;
        params.randomSeed = seedRng;

        std::vector<std::vector<gp_Pnt2d>> cells = VoronoiAlgorithm::generateCellPolygons(params);

        std::vector<Part::Geometry*> newGeom;
        newGeom.reserve(cells.size() * 6);

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

