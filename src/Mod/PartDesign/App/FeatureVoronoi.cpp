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

#include "FeatureVoronoi.h"
#include "VoronoiAlgorithm.h"

#include <Standard_Failure.hxx>
#include <Precision.hxx>

using namespace PartDesign;

static const App::PropertyQuantityConstraint::Constraints quantityRangePositive = {
    2 * Precision::Confusion(), std::numeric_limits<float>::max(), 0.1
};

PROPERTY_SOURCE(PartDesign::FeatureVoronoi, PartDesign::FeaturePrimitive)
PROPERTY_SOURCE(PartDesign::AdditiveVoronoi, PartDesign::FeatureVoronoi)
PROPERTY_SOURCE(PartDesign::SubtractiveVoronoi, PartDesign::FeatureVoronoi)

FeatureVoronoi::FeatureVoronoi()
{
    ADD_PROPERTY_TYPE(NumberOfSeeds, (50), "Voronoi", App::Prop_None, "Number of cell seeds");
    ADD_PROPERTY_TYPE(WallThickness, (2.0f), "Voronoi", App::Prop_None, "Thickness of ribs between cells");
    ADD_PROPERTY_TYPE(FilletRadius, (0.5f), "Voronoi", App::Prop_None, "Corner fillet radius inside cells");
    ADD_PROPERTY_TYPE(Depth, (5.0f), "Voronoi", App::Prop_None, "Extrusion depth of the Voronoi pattern");
    ADD_PROPERTY_TYPE(Width, (100.0f), "Voronoi", App::Prop_None, "Width of the bounding domain");
    ADD_PROPERTY_TYPE(Height, (100.0f), "Voronoi", App::Prop_None, "Height of the bounding domain");
    ADD_PROPERTY_TYPE(SeedRandom, (1234), "Voronoi", App::Prop_None, "Random seed for reproduction");

    WallThickness.setConstraints(&quantityRangePositive);
    Depth.setConstraints(&quantityRangePositive);
    Width.setConstraints(&quantityRangePositive);
    Height.setConstraints(&quantityRangePositive);
}

void FeatureVoronoi::onChanged(const App::Property* prop)
{
    FeaturePrimitive::onChanged(prop);
}

short FeatureVoronoi::mustExecute() const
{
    if (NumberOfSeeds.isTouched() || WallThickness.isTouched() || FilletRadius.isTouched()
        || Depth.isTouched() || Width.isTouched() || Height.isTouched()
        || SeedRandom.isTouched()) {
        return 1;
    }
    return FeaturePrimitive::mustExecute();
}

App::DocumentObjectExecReturn* FeatureVoronoi::execute()
{
    double w = Width.getValue();
    double h = Height.getValue();
    double d = Depth.getValue();
    double t = WallThickness.getValue();

    if (w < Precision::Confusion() || h < Precision::Confusion() || d < Precision::Confusion()) {
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
        params.numPoints = std::max(3, static_cast<int>(NumberOfSeeds.getValue()));
        params.wallThickness = t;
        params.filletRadius = FilletRadius.getValue();
        params.depth = d;
        params.width = w;
        params.height = h;
        params.randomSeed = SeedRandom.getValue();
        params.isAdditive = (getAddSubType() == FeatureAddSub::Type::Additive);

        TopoDS_Shape voronoiShape = VoronoiAlgorithm::generateVoronoi(params);
        if (voronoiShape.IsNull()) {
            return new App::DocumentObjectExecReturn(
                QT_TRANSLATE_NOOP("Exception", "Failed to generate Voronoi shape")
            );
        }

        return FeaturePrimitive::execute(voronoiShape);
    }
    catch (Standard_Failure& e) {
        return new App::DocumentObjectExecReturn(e.GetMessageString());
    }
}
