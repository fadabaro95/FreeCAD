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

#include <QApplication>
#include <QDialog>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QDialogButtonBox>
#include <QLabel>
#include <QCheckBox>
#include <random>
#include <algorithm>

#include <App/Document.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include <Gui/MainWindow.h>
#include <Gui/Selection/Selection.h>
#include <App/Origin.h>

#include <Mod/PartDesign/App/Body.h>
#include <Mod/PartDesign/App/FeatureVoronoi.h>
#include <Mod/PartDesign/App/VoronoiSketch.h>
#include <Mod/PartDesign/App/VoronoiAlgorithm.h>
#include <Mod/Sketcher/App/SketchObject.h>
#include <Mod/Part/App/Geometry.h>

#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>

#include "Utils.h"

using namespace PartDesignGui;

namespace {

class DlgVoronoiSketch : public QDialog
{
public:
    explicit DlgVoronoiSketch(QWidget* parent = nullptr)
        : QDialog(parent)
    {
        setWindowTitle(tr("Voronoi Sketch Parameters"));
        setMinimumWidth(320);

        auto* mainLayout = new QVBoxLayout(this);
        auto* form = new QFormLayout();

        spinSeeds = new QSpinBox(this);
        spinSeeds->setRange(3, 1000);
        spinSeeds->setValue(50);
        form->addRow(tr("Number of seeds:"), spinSeeds);

        spinWallThickness = new QDoubleSpinBox(this);
        spinWallThickness->setRange(0.01, 100.0);
        spinWallThickness->setValue(2.0);
        spinWallThickness->setDecimals(2);
        spinWallThickness->setSuffix(QStringLiteral(" mm"));
        form->addRow(tr("Wall thickness:"), spinWallThickness);

        spinFilletRadius = new QDoubleSpinBox(this);
        spinFilletRadius->setRange(0.0, 50.0);
        spinFilletRadius->setValue(0.5);
        spinFilletRadius->setDecimals(2);
        spinFilletRadius->setSuffix(QStringLiteral(" mm"));
        form->addRow(tr("Fillet radius:"), spinFilletRadius);

        spinWidth = new QDoubleSpinBox(this);
        spinWidth->setRange(1.0, 100000.0);
        spinWidth->setValue(100.0);
        spinWidth->setDecimals(2);
        spinWidth->setSuffix(QStringLiteral(" mm"));
        form->addRow(tr("Width:"), spinWidth);

        spinHeight = new QDoubleSpinBox(this);
        spinHeight->setRange(1.0, 100000.0);
        spinHeight->setValue(100.0);
        spinHeight->setDecimals(2);
        spinHeight->setSuffix(QStringLiteral(" mm"));
        form->addRow(tr("Height:"), spinHeight);

        auto* seedLayout = new QHBoxLayout();
        spinRandomSeed = new QSpinBox(this);
        spinRandomSeed->setRange(1, 999999);
        spinRandomSeed->setValue(1234);

        auto* btnRandom = new QPushButton(tr("Randomize"), this);
        seedLayout->addWidget(spinRandomSeed);
        seedLayout->addWidget(btnRandom);
        form->addRow(tr("Random seed:"), seedLayout);

        checkCloseContour = new QCheckBox(tr("Close outer contour"), this);
        checkCloseContour->setChecked(true);
        form->addRow(QString(), checkCloseContour);

        mainLayout->addLayout(form);

        auto* btnBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
        mainLayout->addWidget(btnBox);

        connect(btnBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
        connect(btnBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
        connect(btnRandom, &QPushButton::clicked, [this]() {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(1, 999999);
            spinRandomSeed->setValue(dis(gen));
        });
    }

    void setDimensions(double w, double h)
    {
        if (w > 1.0) spinWidth->setValue(w);
        if (h > 1.0) spinHeight->setValue(h);
    }

    PartDesign::VoronoiParams getParams() const
    {
        PartDesign::VoronoiParams params;
        params.numPoints = spinSeeds->value();
        params.wallThickness = spinWallThickness->value();
        params.filletRadius = spinFilletRadius->value();
        params.width = spinWidth->value();
        params.height = spinHeight->value();
        params.randomSeed = spinRandomSeed->value();
        params.closeContour = checkCloseContour->isChecked();
        return params;
    }

private:
    QSpinBox* spinSeeds {nullptr};
    QDoubleSpinBox* spinWallThickness {nullptr};
    QDoubleSpinBox* spinFilletRadius {nullptr};
    QDoubleSpinBox* spinWidth {nullptr};
    QDoubleSpinBox* spinHeight {nullptr};
    QSpinBox* spinRandomSeed {nullptr};
    QCheckBox* checkCloseContour {nullptr};
};

} // namespace

DEF_STD_CMD_A(CmdPartDesignAdditiveVoronoi)

CmdPartDesignAdditiveVoronoi::CmdPartDesignAdditiveVoronoi()
    : Command("PartDesign_AdditiveVoronoi")
{
    sGroup        = "PartDesign";
    sMenuText     = QT_TR_NOOP("Additive Voronoi...");
    sToolTipText  = QT_TR_NOOP("Create an additive Voronoi pattern attached to a face");
    sWhatsThis    = "PartDesign_AdditiveVoronoi";
    sStatusTip    = QT_TR_NOOP("Create an additive Voronoi pattern attached to a face");
    sPixmap       = "PartDesign_CompPrimitiveAdditive";
}

void CmdPartDesignAdditiveVoronoi::activated(int /*iMsg*/)
{
    PartDesign::Body* pcActiveBody = PartDesignGui::getBody(true);
    if (!pcActiveBody) return;

    openCommand("Create Additive Voronoi");

    auto featName = getUniqueObjectName("AdditiveVoronoi", pcActiveBody);
    FCMD_OBJ_DOC_CMD(pcActiveBody, "addObject('PartDesign::AdditiveVoronoi','" << featName << "')");

    auto* prm = static_cast<PartDesign::FeatureVoronoi*>(
        pcActiveBody->getDocument()->getObject(featName.c_str())
    );
    if (!prm) return;

    FCMD_OBJ_CMD(pcActiveBody, "addObject(" << getObjectCmd(prm) << ")");
    Gui::Command::updateActive();

    auto base = prm->BaseFeature.getValue();
    FCMD_OBJ_HIDE(base);

    PartDesignGui::setEdit(prm, pcActiveBody);
}

bool CmdPartDesignAdditiveVoronoi::isActive()
{
    return (hasActiveDocument() && !Gui::Control().activeDialog());
}

DEF_STD_CMD_A(CmdPartDesignSubtractiveVoronoi)

CmdPartDesignSubtractiveVoronoi::CmdPartDesignSubtractiveVoronoi()
    : Command("PartDesign_SubtractiveVoronoi")
{
    sGroup        = "PartDesign";
    sMenuText     = QT_TR_NOOP("Subtractive Voronoi...");
    sToolTipText  = QT_TR_NOOP("Create a subtractive Voronoi pattern attached to a face");
    sWhatsThis    = "PartDesign_SubtractiveVoronoi";
    sStatusTip    = QT_TR_NOOP("Create a subtractive Voronoi pattern attached to a face");
    sPixmap       = "PartDesign_CompPrimitiveSubtractive";
}

void CmdPartDesignSubtractiveVoronoi::activated(int /*iMsg*/)
{
    PartDesign::Body* pcActiveBody = PartDesignGui::getBody(true);
    if (!pcActiveBody) return;

    openCommand("Create Subtractive Voronoi");

    auto featName = getUniqueObjectName("SubtractiveVoronoi", pcActiveBody);
    FCMD_OBJ_DOC_CMD(pcActiveBody, "addObject('PartDesign::SubtractiveVoronoi','" << featName << "')");

    auto* prm = static_cast<PartDesign::FeatureVoronoi*>(
        pcActiveBody->getDocument()->getObject(featName.c_str())
    );
    if (!prm) return;

    FCMD_OBJ_CMD(pcActiveBody, "addObject(" << getObjectCmd(prm) << ")");
    Gui::Command::updateActive();

    auto base = prm->BaseFeature.getValue();
    FCMD_OBJ_HIDE(base);

    PartDesignGui::setEdit(prm, pcActiveBody);
}

bool CmdPartDesignSubtractiveVoronoi::isActive()
{
    return (hasActiveDocument() && !Gui::Control().activeDialog());
}

DEF_STD_CMD_A(CmdPartDesignVoronoiSketch)

CmdPartDesignVoronoiSketch::CmdPartDesignVoronoiSketch()
    : Command("PartDesign_VoronoiSketch")
{
    sGroup        = "PartDesign";
    sMenuText     = QT_TR_NOOP("Voronoi Sketch...");
    sToolTipText  = QT_TR_NOOP("Create a Voronoi sketch attached to a face or active Body");
    sWhatsThis    = "PartDesign_VoronoiSketch";
    sStatusTip    = QT_TR_NOOP("Create a Voronoi sketch attached to a face or active Body");
    sPixmap       = "Sketcher_NewSketch";
}

void CmdPartDesignVoronoiSketch::activated(int /*iMsg*/)
{
    PartDesign::Body* pcActiveBody = PartDesignGui::getBody(true);
    if (!pcActiveBody) return;

    // Check selection for planar support and dimensions
    std::vector<Gui::SelectionObject> sel = Gui::Selection().getSelectionEx();
    std::string supportStr;
    double initW = 100.0;
    double initH = 100.0;

    for (const auto& selObj : sel) {
        if (selObj.getObject() && selObj.hasSubNames()) {
            const auto& subNames = selObj.getSubNames();
            if (!subNames.empty() && subNames[0].rfind("Face", 0) == 0) {
                supportStr = selObj.getAsPropertyLinkSubString();
                const auto* feat = selObj.getObject<Part::Feature>();
                if (feat) {
                    TopoDS_Shape sub = feat->Shape.getShape().getSubShape(subNames[0].c_str());
                    if (!sub.IsNull() && sub.ShapeType() == TopAbs_FACE) {
                        Bnd_Box b;
                        BRepBndLib::Add(sub, b);
                        if (!b.IsVoid()) {
                            double xmin, ymin, zmin, xmax, ymax, zmax;
                            b.Get(xmin, ymin, zmin, xmax, ymax, zmax);
                            double dims[3] = {xmax - xmin, ymax - ymin, zmax - zmin};
                            std::sort(dims, dims + 3);
                            if (dims[1] > 1.0 && dims[2] > 1.0) {
                                initW = std::round(dims[2]);
                                initH = std::round(dims[1]);
                            }
                        }
                    }
                }
                break;
            }
        }
    }

    DlgVoronoiSketch dlg(Gui::getMainWindow());
    dlg.setDimensions(initW, initH);
    if (dlg.exec() != QDialog::Accepted) {
        return;
    }

    PartDesign::VoronoiParams params = dlg.getParams();

    openCommand(QT_TRANSLATE_NOOP("Command", "Create Voronoi Sketch"));

    App::Document* doc = pcActiveBody->getDocument();
    std::string featName = doc->getUniqueObjectName("VoronoiSketch");
    FCMD_OBJ_CMD(pcActiveBody, "newObject('PartDesign::VoronoiSketch','" << featName << "')");

    auto* sketchObj = static_cast<PartDesign::VoronoiSketch*>(doc->getObject(featName.c_str()));
    if (!sketchObj) {
        abortCommand();
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

    // Attachment
    if (!supportStr.empty()) {
        FCMD_OBJ_CMD(sketchObj, "AttachmentSupport = " << supportStr);
        FCMD_OBJ_CMD(sketchObj, "MapMode = 'FlatFace'");
    }
    else {
        auto origin = pcActiveBody->getOrigin();
        if (origin && origin->getXY()) {
            FCMD_OBJ_CMD(sketchObj, "AttachmentSupport = [(" << Gui::Command::getObjectCmd(origin->getXY()) << ", '')]");
            FCMD_OBJ_CMD(sketchObj, "MapMode = 'FlatFace'");
        }
    }

    sketchObj->recomputeFeature();
    commitCommand();
    Gui::Command::updateActive();
}

bool CmdPartDesignVoronoiSketch::isActive()
{
    return (hasActiveDocument() && !Gui::Control().activeDialog());
}

void CreatePartDesignVoronoiCommands()
{
    Gui::CommandManager& rcCmdMgr = Gui::Application::Instance->commandManager();
    rcCmdMgr.addCommand(new CmdPartDesignAdditiveVoronoi);
    rcCmdMgr.addCommand(new CmdPartDesignSubtractiveVoronoi);
    rcCmdMgr.addCommand(new CmdPartDesignVoronoiSketch);
}
