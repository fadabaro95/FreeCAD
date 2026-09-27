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

#include <QMenu>
#include <Gui/BitmapFactory.h>
#include <Mod/PartDesign/App/FeatureVoronoi.h>

#include "ViewProviderVoronoi.h"
#include "TaskVoronoiParameters.h"

using namespace PartDesignGui;

PROPERTY_SOURCE(PartDesignGui::ViewProviderVoronoi, PartDesignGui::ViewProviderPrimitive)

ViewProviderVoronoi::ViewProviderVoronoi() = default;

ViewProviderVoronoi::~ViewProviderVoronoi() = default;

void ViewProviderVoronoi::setupContextMenu(QMenu* menu, QObject* receiver, const char* member)
{
    addDefaultAction(menu, QObject::tr("Edit Voronoi"));
    PartDesignGui::ViewProvider::setupContextMenu(menu, receiver, member);
}

TaskDlgFeatureParameters* ViewProviderVoronoi::getEditDialog()
{
    return new TaskDlgVoronoiParameters(this);
}

QIcon ViewProviderVoronoi::getIcon() const
{
    return Gui::BitmapFactory().iconFromTheme("PartDesign_Voronoi");
}
