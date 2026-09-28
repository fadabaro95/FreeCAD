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

#include <Mod/Sketcher/App/SketchObject.h>
#include <App/PropertyStandard.h>
#include <App/PropertyUnits.h>
#include <App/PropertyLinks.h>
#include <Mod/PartDesign/PartDesignGlobal.h>

namespace PartDesign
{

/**
 * @brief Fully parametric 2D Voronoi sketch.
 *
 * Inherits from Sketcher::SketchObject so that it can be used directly as profile
 * by PartDesign Pad, Pocket, etc. Regenerates its cell geometry whenever any
 * Voronoi parameter (NumberOfSeeds, WallThickness, FilletRadius, Width, Height,
 * SeedRandom, BoundarySketch) is modified.
 */
class PartDesignExport VoronoiSketch : public Sketcher::SketchObject
{
    PROPERTY_HEADER_WITH_OVERRIDE(PartDesign::VoronoiSketch);

public:
    VoronoiSketch();
    ~VoronoiSketch() override = default;

    App::PropertyInteger NumberOfSeeds;
    App::PropertyLength  WallThickness;
    App::PropertyLength  FilletRadius;
    App::PropertyLength  Width;
    App::PropertyLength  Height;
    App::PropertyInteger SeedRandom;
    App::PropertyBool    CloseContour;
    App::PropertyLink    BoundarySketch;

    short mustExecute() const override;
    App::DocumentObjectExecReturn* execute() override;
    void onChanged(const App::Property* prop) override;
};

} // namespace PartDesign

