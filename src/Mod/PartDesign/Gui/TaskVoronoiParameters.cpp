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

#include "TaskVoronoiParameters.h"
#include "ui_TaskVoronoiParameters.h"

#include <random>
#include <QDialogButtonBox>
#include <App/Document.h>
#include <App/Origin.h>
#include <Gui/Application.h>
#include <Gui/Command.h>
#include <Gui/Document.h>
#include <Mod/PartDesign/App/Body.h>
#include <Mod/PartDesign/App/FeatureVoronoi.h>
#include <Mod/PartDesign/App/VoronoiSketch.h>
#include <Mod/PartDesign/App/VoronoiAlgorithm.h>
#include <Mod/Sketcher/App/SketchObject.h>
#include <Mod/Part/App/Geometry.h>
#include "Utils.h"

using namespace PartDesignGui;

TaskBoxVoronoi::TaskBoxVoronoi(ViewProviderVoronoi* vp, QWidget* parent)
    : TaskFeatureParameters(vp, parent, "PartDesign_Voronoi", tr("Voronoi Parameters"))
    , ui(std::make_unique<Ui_TaskVoronoiParameters>())
{
    QWidget* proxy = new QWidget(this);
    ui->setupUi(proxy);
    this->groupLayout()->addWidget(proxy);

    PartDesign::FeatureVoronoi* obj = getVoronoiObject();
    if (obj) {
        ui->spinSeeds->setValue(static_cast<int>(obj->NumberOfSeeds.getValue()));
        ui->spinRandomSeed->setValue(static_cast<int>(obj->SeedRandom.getValue()));

        ui->spinWallThickness->bind(obj->WallThickness);
        ui->spinFilletRadius->bind(obj->FilletRadius);
        ui->spinDepth->bind(obj->Depth);
        ui->spinWidth->bind(obj->Width);
        ui->spinHeight->bind(obj->Height);
    }

    connect(ui->spinSeeds, qOverload<int>(&QSpinBox::valueChanged),
            this, &TaskBoxVoronoi::onSeedsChanged);
    connect(ui->spinRandomSeed, qOverload<int>(&QSpinBox::valueChanged),
            this, &TaskBoxVoronoi::onRandomSeedChanged);
    connect(ui->buttonRandomize, &QPushButton::clicked,
            this, &TaskBoxVoronoi::onRandomizeClicked);

    connect(ui->spinWallThickness, qOverload<const Base::Quantity&>(&Gui::QuantitySpinBox::valueChanged),
            this, &TaskBoxVoronoi::onWallThicknessChanged);
    connect(ui->spinFilletRadius, qOverload<const Base::Quantity&>(&Gui::QuantitySpinBox::valueChanged),
            this, &TaskBoxVoronoi::onFilletRadiusChanged);
    connect(ui->spinDepth, qOverload<const Base::Quantity&>(&Gui::QuantitySpinBox::valueChanged),
            this, &TaskBoxVoronoi::onDepthChanged);
    connect(ui->spinWidth, qOverload<const Base::Quantity&>(&Gui::QuantitySpinBox::valueChanged),
            this, &TaskBoxVoronoi::onWidthChanged);
    connect(ui->spinHeight, qOverload<const Base::Quantity&>(&Gui::QuantitySpinBox::valueChanged),
            this, &TaskBoxVoronoi::onHeightChanged);
    connect(ui->buttonCreateSketch, &QPushButton::clicked,
            this, &TaskBoxVoronoi::onCreateSketchClicked);
}

TaskBoxVoronoi::~TaskBoxVoronoi() = default;

PartDesign::FeatureVoronoi* TaskBoxVoronoi::getVoronoiObject() const
{
    return vp ? vp->getObject<PartDesign::FeatureVoronoi>() : nullptr;
}

void TaskBoxVoronoi::apply()
{
    PartDesign::FeatureVoronoi* obj = getVoronoiObject();
    if (!obj) return;

    ui->spinWallThickness->apply();
    ui->spinFilletRadius->apply();
    ui->spinDepth->apply();
    ui->spinWidth->apply();
    ui->spinHeight->apply();
    obj->NumberOfSeeds.setValue(ui->spinSeeds->value());
    obj->SeedRandom.setValue(ui->spinRandomSeed->value());
}

void TaskBoxVoronoi::onSeedsChanged(int value)
{
    PartDesign::FeatureVoronoi* obj = getVoronoiObject();
    if (obj && !isUpdateBlocked()) {
        obj->NumberOfSeeds.setValue(value);
        recomputeFeature();
    }
}

void TaskBoxVoronoi::onWallThicknessChanged(const Base::Quantity& /*val*/)
{
    if (!isUpdateBlocked()) {
        recomputeFeature();
    }
}

void TaskBoxVoronoi::onFilletRadiusChanged(const Base::Quantity& /*val*/)
{
    if (!isUpdateBlocked()) {
        recomputeFeature();
    }
}

void TaskBoxVoronoi::onDepthChanged(const Base::Quantity& /*val*/)
{
    if (!isUpdateBlocked()) {
        recomputeFeature();
    }
}

void TaskBoxVoronoi::onWidthChanged(const Base::Quantity& /*val*/)
{
    if (!isUpdateBlocked()) {
        recomputeFeature();
    }
}

void TaskBoxVoronoi::onHeightChanged(const Base::Quantity& /*val*/)
{
    if (!isUpdateBlocked()) {
        recomputeFeature();
    }
}

void TaskBoxVoronoi::onRandomSeedChanged(int value)
{
    PartDesign::FeatureVoronoi* obj = getVoronoiObject();
    if (obj && !isUpdateBlocked()) {
        obj->SeedRandom.setValue(value);
        recomputeFeature();
    }
}

void TaskBoxVoronoi::onRandomizeClicked()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dist(1, 999999);
    ui->spinRandomSeed->setValue(dist(gen));
}

void TaskBoxVoronoi::onCreateSketchClicked()
{
    PartDesign::Body* pcActiveBody = PartDesignGui::getBody(true);
    if (!pcActiveBody) return;

    PartDesign::FeatureVoronoi* obj = getVoronoiObject();

    PartDesign::VoronoiParams params;
    params.numPoints = ui->spinSeeds->value();
    params.wallThickness = ui->spinWallThickness->value().getValue();
    params.filletRadius = ui->spinFilletRadius->value().getValue();
    params.width = ui->spinWidth->value().getValue();
    params.height = ui->spinHeight->value().getValue();
    params.randomSeed = ui->spinRandomSeed->value();
    params.closeContour = ui->checkCloseContour->isChecked();

    App::Document* doc = pcActiveBody->getDocument();
    doc->openTransaction("Create Voronoi Sketch");

    std::string featName = doc->getUniqueObjectName("VoronoiSketch");
    FCMD_OBJ_CMD(pcActiveBody, "newObject('PartDesign::VoronoiSketch','" << featName << "')");

    auto* sketchObj = static_cast<PartDesign::VoronoiSketch*>(doc->getObject(featName.c_str()));
    if (!sketchObj) {
        doc->abortTransaction();
        return;
    }

    FCMD_OBJ_CMD(sketchObj, "Label = 'VoronoiSketch'");
    FCMD_OBJ_CMD(sketchObj, "NumberOfSeeds = " << params.numPoints);
    FCMD_OBJ_CMD(sketchObj, "WallThickness = " << params.wallThickness);
    FCMD_OBJ_CMD(sketchObj, "FilletRadius = " << params.filletRadius);
    FCMD_OBJ_CMD(sketchObj, "Width = " << params.width);
    FCMD_OBJ_CMD(sketchObj, "Height = " << params.height);
    FCMD_OBJ_CMD(sketchObj, "SeedRandom = " << params.randomSeed);
    FCMD_OBJ_CMD(sketchObj, "CloseContour = " << (params.closeContour ? "True" : "False"));

    // Match attachment of the Voronoi feature or Origin XY
    if (obj) {
        std::string sup = obj->AttachmentSupport.getPyReprString();
        if (sup != "[]" && !sup.empty()) {
            FCMD_OBJ_CMD(sketchObj, "AttachmentSupport = " << sup);
            FCMD_OBJ_CMD(sketchObj, "MapMode = '" << obj->MapMode.getValueAsString() << "'");
        }
        else {
            auto origin = pcActiveBody->getOrigin();
            if (origin && origin->getXY()) {
                FCMD_OBJ_CMD(sketchObj, "AttachmentSupport = [(" << Gui::Command::getObjectCmd(origin->getXY()) << ", '')]");
                FCMD_OBJ_CMD(sketchObj, "MapMode = 'FlatFace'");
            }
        }
    }

    sketchObj->recomputeFeature();
    doc->commitTransaction();
    Gui::Command::updateActive();
}

// --------------------------------------------------------------------------
// TaskDlgVoronoiParameters
// --------------------------------------------------------------------------

TaskDlgVoronoiParameters::TaskDlgVoronoiParameters(ViewProviderVoronoi* voronoiView)
    : TaskDlgFeatureParameters(voronoiView)
{
    taskBox = new TaskBoxVoronoi(voronoiView);
    Content.push_back(taskBox);
}

TaskDlgVoronoiParameters::~TaskDlgVoronoiParameters() = default;

QDialogButtonBox::StandardButtons TaskDlgVoronoiParameters::getStandardButtons() const
{
    return QDialogButtonBox::Ok | QDialogButtonBox::Cancel;
}

#include "moc_TaskVoronoiParameters.cpp"
