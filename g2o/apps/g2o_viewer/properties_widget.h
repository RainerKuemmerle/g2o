// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
//
// This file is part of g2o.
// 
// g2o is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// g2o is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with g2o.  If not, see <http://www.gnu.org/licenses/>.

#ifndef G2O_PROPERTIES_WINDOW_H
#define G2O_PROPERTIES_WINDOW_H

#include <vector>
#include <string>

#include <QDialog>

#include "g2o_viewer_api.h"
#include "ui_base_properties_widget.h"

namespace g2o
{
  class G2oQGLViewer;
  class PropertyMap;
}

class G2O_VIEWER_API PropertiesWidget : public QDialog, public Ui::BasePropertiesWidget
{
  Q_OBJECT
  public:
    PropertiesWidget(QWidget * parent = 0, Qt::WindowFlags f = 0);
    virtual ~PropertiesWidget();

    void setProperties(g2o::PropertyMap* properties);

  public slots:
    void on_btnApply_clicked();
    void on_btnOK_clicked();

  protected:
    std::vector<std::string> _propNames;
    g2o::PropertyMap* _properties;

    virtual void updateDisplayedProperties();
    virtual void applyProperties();
    virtual std::string humanReadablePropName(const std::string& propertyName) const;
};

#endif
