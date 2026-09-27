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

#include "FeaturePrimitive.h"
#include <App/PropertyStandard.h>
#include <App/PropertyUnits.h>

namespace PartDesign
{

/**
 * @brief Parametric Voronoi Feature in PartDesign.
 *
 * Can be attached to a planar face or datum plane via AttachExtension,
 * generating generative cellular lattice or ribs that are automatically
 * fused (Additive) or cut (Subtractive) with the active Body solid.
 */
class PartDesignExport FeatureVoronoi: public PartDesign::FeaturePrimitive
{
    PROPERTY_HEADER_WITH_OVERRIDE(PartDesign::FeatureVoronoi);

public:
    FeatureVoronoi();

    App::PropertyInteger           NumberOfSeeds;
    App::PropertyLength            WallThickness;
    App::PropertyLength            FilletRadius;
    App::PropertyLength            Depth;
    App::PropertyLength            Width;
    App::PropertyLength            Height;
    App::PropertyInteger           SeedRandom;

    App::DocumentObjectExecReturn* execute() override;
    short mustExecute() const override;
    void onChanged(const App::Property* prop) override;

    const char* getViewProviderName() const override
    {
        return "PartDesignGui::ViewProviderVoronoi";
    }
};

class PartDesignExport AdditiveVoronoi: public FeatureVoronoi
{
    PROPERTY_HEADER_WITH_OVERRIDE(PartDesign::AdditiveVoronoi);

public:
    AdditiveVoronoi()
    {
        defineAdditive();
    }
};

class PartDesignExport SubtractiveVoronoi: public FeatureVoronoi
{
    PROPERTY_HEADER_WITH_OVERRIDE(PartDesign::SubtractiveVoronoi);

public:
    SubtractiveVoronoi()
    {
        defineSubtractive();
    }
};

} // namespace PartDesign
