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

#include "ViewProviderPrimitive.h"

namespace PartDesignGui
{

class PartDesignGuiExport ViewProviderVoronoi: public ViewProviderPrimitive
{
    PROPERTY_HEADER_WITH_OVERRIDE(PartDesignGui::ViewProviderVoronoi);

public:
    ViewProviderVoronoi();
    ~ViewProviderVoronoi() override;

protected:
    QIcon getIcon() const override;
    void setupContextMenu(QMenu* menu, QObject* receiver, const char* member) override;
    TaskDlgFeatureParameters* getEditDialog() override;
};

} // namespace PartDesignGui
