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

#include "properties_widget.h"

#include <QLineEdit>

#include <iostream>
#include <cassert>

#include "g2o/stuff/property.h"

using namespace std;

using namespace g2o;

PropertiesWidget::PropertiesWidget(QWidget * parent, Qt::WindowFlags f) :
  QDialog(parent, f),
  _properties(0)
{
  setupUi(this);
}

PropertiesWidget::~PropertiesWidget()
{
}

void PropertiesWidget::updateDisplayedProperties()
{
  tableWidget->clear();
  _propNames.clear();

  tableWidget->setColumnCount(2);

  QStringList horizontalHeaders;
  horizontalHeaders.append("Name");
  horizontalHeaders.append("Value");
  tableWidget->setHorizontalHeaderLabels(horizontalHeaders);

  tableWidget->verticalHeader()->hide();

  PropertyMap* properties = _properties;
  if (! properties)
    return;
  tableWidget->setRowCount(properties->size());

  int r = 0;
  for (PropertyMap::PropertyMapIterator it = properties->begin(); it != properties->end(); ++it, ++r) {

    QTableWidgetItem* textItem = new QTableWidgetItem;
    textItem->setText(QString::fromStdString(humanReadablePropName(it->first)));
    textItem->setFlags(textItem->flags() & ~Qt::ItemIsEditable);
    tableWidget->setItem(r, 0, textItem);
    _propNames.push_back(it->first);

    if (dynamic_cast<Property<bool>*>(it->second)) {
      Property<bool>* prop = static_cast<Property<bool>*>(it->second);
      QTableWidgetItem* checkItem = new QTableWidgetItem;
      checkItem->setText("enabled");
      checkItem->setFlags(checkItem->flags() | Qt::ItemIsUserCheckable);
      if (prop->value())
        checkItem->setCheckState(Qt::Checked);
      else
        checkItem->setCheckState(Qt::Unchecked);
      tableWidget->setItem(r, 1, checkItem);
    } else {
      QLineEdit* editor = new QLineEdit(tableWidget);
      editor->setText(QString::fromStdString(it->second->toString()));
      if (dynamic_cast<Property<int>*>(it->second)) {
        editor->setValidator(new QIntValidator(editor));
      }
      else if (dynamic_cast<Property<float>*>(it->second)) {
        editor->setValidator(new QDoubleValidator(editor));
      }
      else if (dynamic_cast<Property<double>*>(it->second)) {
        editor->setValidator(new QDoubleValidator(editor));
      }
      tableWidget->setCellWidget(r, 1, editor);
    }

  }
  tableWidget->resizeColumnToContents(0);
}

void PropertiesWidget::applyProperties()
{
  assert(tableWidget->rowCount() == (int) _propNames.size());
  PropertyMap* properties = _properties;
  for (int r = 0; r < tableWidget->rowCount(); ++r) {
    const std::string& propName = _propNames[r];
    BaseProperty* baseProp = properties->getProperty<BaseProperty>(propName);
    if (! baseProp)
      continue;

    if (dynamic_cast<Property<bool>*>(baseProp)) {
      Property<bool>* prop = static_cast<Property<bool>*>(baseProp);
      QTableWidgetItem* checkItem = tableWidget->item(r, 1);
      prop->setValue(checkItem->checkState() == Qt::Checked);
    } else {
      QLineEdit* editor = dynamic_cast<QLineEdit*>(tableWidget->cellWidget(r, 1));
      bool status = baseProp->fromString(editor->text().toStdString());
      if (! status) {
        cerr << "Warning: unable to set property " << baseProp->name() << endl;
      }
    }
  }
}

void PropertiesWidget::on_btnApply_clicked()
{
  applyProperties();
}

void PropertiesWidget::on_btnOK_clicked()
{
  applyProperties();
  close();
}

std::string PropertiesWidget::humanReadablePropName(const std::string& propertyName) const
{
  return propertyName;
}

void PropertiesWidget::setProperties(PropertyMap* properties)
{
  _properties = properties;
  updateDisplayedProperties();
}
