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

#include <memory>
#include <Base/Quantity.h>
#include "TaskFeatureParameters.h"
#include "ViewProviderVoronoi.h"

namespace PartDesign
{
class FeatureVoronoi;
}

namespace PartDesignGui
{

class Ui_TaskVoronoiParameters;

class TaskBoxVoronoi: public TaskFeatureParameters
{
    Q_OBJECT

public:
    explicit TaskBoxVoronoi(ViewProviderVoronoi* vp, QWidget* parent = nullptr);
    ~TaskBoxVoronoi() override;

    void apply() override;

public Q_SLOTS:
    void onSeedsChanged(int value);
    void onWallThicknessChanged(const Base::Quantity& val);
    void onFilletRadiusChanged(const Base::Quantity& val);
    void onDepthChanged(const Base::Quantity& val);
    void onWidthChanged(const Base::Quantity& val);
    void onHeightChanged(const Base::Quantity& val);
    void onRandomSeedChanged(int value);
    void onRandomizeClicked();
    void onCreateSketchClicked();

private:
    PartDesign::FeatureVoronoi* getVoronoiObject() const;
    std::unique_ptr<Ui_TaskVoronoiParameters> ui;
};

class TaskDlgVoronoiParameters: public TaskDlgFeatureParameters
{
    Q_OBJECT

public:
    explicit TaskDlgVoronoiParameters(ViewProviderVoronoi* voronoiView);
    ~TaskDlgVoronoiParameters() override;

    QDialogButtonBox::StandardButtons getStandardButtons() const override;

private:
    TaskBoxVoronoi* taskBox = nullptr;
};

} // namespace PartDesignGui
